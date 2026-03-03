// robotTeleopRx.cpp
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <sstream>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <mutex>
#include <chrono>

std::mutex rxMutex;
std::string latestRx;     // last received line (already formatted)
bool rxDirty = false;     // new data arrived since last print

static int clampi(int v, int lo, int hi) {
    return std::max(lo, std::min(hi, v));
}

static bool set_interface_attribs(int fd, int baud) {
    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "tcgetattr: " << strerror(errno) << "\n";
        return false;
    }

    
    cfmakeraw(&tty);

    speed_t speed = B115200;
    if (baud == 9600) speed = B9600;
    else if (baud == 57600) speed = B57600;
    else if (baud == 115200) speed = B115200;
    else {
        std::cerr << "Unsupported baud\n";
        return false;
    }

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    // 8N1, no flow control
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    // Non-blocking-ish reads: timeout 0.2s
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 2;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "tcsetattr: " << strerror(errno) << "\n";
        return false;
    }
    return true;
}

struct MotorCmd {
    int pwmR = 0;   // 0..100
    int pwmL = 0;   // 0..100
    int dirR = 1;   // 0/1
    int dirL = 1;   // 0/1
};

static bool write_motor(int fd, const MotorCmd& c) {
    int pwmR = clampi(c.pwmR, 0, 100);
    int pwmL = clampi(c.pwmL, 0, 100);
    int dirR = c.dirR ? 1 : 0;
    int dirL = c.dirL ? 1 : 0;

    std::string msg = std::to_string(pwmR) + "," +
                      std::to_string(pwmL) + "," +
                      std::to_string(dirR) + "," +
                      std::to_string(dirL) + "\n";

    ssize_t n = write(fd, msg.data(), msg.size());
    if (n < 0) {
        std::cerr << "write: " << strerror(errno) << "\n";
        return false;
    }
    tcdrain(fd);
    return true;
}

// Terminal raw mode for single key press
struct TermRawGuard {
    termios orig{};
    bool ok = false;

    TermRawGuard() {
        if (tcgetattr(STDIN_FILENO, &orig) == 0) {
            termios raw = orig;
            cfmakeraw(&raw);
            raw.c_cc[VMIN]  = 0;
            raw.c_cc[VTIME] = 0;
            if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) ok = true;
        }
    }
    ~TermRawGuard() {
        if (ok) tcsetattr(STDIN_FILENO, TCSANOW, &orig);
    }
};

static int read_key_nonblock() {
    unsigned char ch;
    ssize_t n = read(STDIN_FILENO, &ch, 1);
    if (n == 1) return (int)ch;
    return -1;
}

// Read one line from UART (timeout based on VTIME)
static bool read_line(int fd, std::string &out) {
    out.clear();
    char ch;
    while (true) {
        ssize_t n = read(fd, &ch, 1);
        if (n == 0) {
            // timeout
            return !out.empty();
        }
        if (n < 0) {
            if (errno == EAGAIN) return !out.empty();
            std::cerr << "read: " << strerror(errno) << "\n";
            return false;
        }
        if (ch == '\n') return true;
        if (ch != '\r') out.push_back(ch);
        if (out.size() > 4096) return true;
    }
}

static bool parse_robot_info(const std::string& line,
                             int &leftPct, int &rightPct,
                             int &encR, int &encL, int &encLS)
{
    std::stringstream ss(line);
    std::string tok;
    std::vector<int> v;

    while (std::getline(ss, tok, ',')) {
        tok.erase(0, tok.find_first_not_of(" \t"));
        if (!tok.empty())
            tok.erase(tok.find_last_not_of(" \t") + 1);

        try {
            v.push_back(std::stoi(tok));
        } catch (...) {
            return false;
        }
    }

    if (v.size() != 5) return false;
    leftPct  = v[0];
    rightPct = v[1];
    encR     = v[2];
    encL     = v[3];
    encLS    = v[4];
    return true;
}

int main(int argc, char** argv) {
    const char *PORT = (argc >= 2) ? argv[1] : "/dev/ttyUSB0"; //with argment ./robotCtl /dev/ttyUSB1 or default /dev/ttyUSB0
    const int BAUD = 115200;

    int fd = open(PORT, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "open(" << PORT << "): " << strerror(errno) << "\n";
        return 1;
    }
    if (!set_interface_attribs(fd, BAUD)) {
        close(fd);
        return 1;
    }

    // Give ESP32 time (optional)
    usleep(500 * 1000);

    TermRawGuard tg;
    if (!tg.ok) {
        std::cerr << "Failed to set terminal raw mode\n";
        close(fd);
        return 1;
    }

    std::atomic<bool> running{true};

    //RX thread: permanently read + parse
    std::thread rxThread([&](){
        while (running.load()) {
            std::string line;
            if (!read_line(fd, line)) continue;
            if (line.empty()) continue;

            int leftPct, rightPct, encR, encL, encLS;
            std::string formatted;

            if (parse_robot_info(line, leftPct, rightPct, encR, encL, encLS)) {
                formatted = "RX left=" + std::to_string(leftPct) +
                            "% right=" + std::to_string(rightPct) +
                            "% encR=" + std::to_string(encR) +
                            " encL=" + std::to_string(encL) +
                            " encLS=" + std::to_string(encLS);
            } else {
                formatted = "RX " + line;
            }

            {
                std::lock_guard<std::mutex> lk(rxMutex);
                latestRx = std::move(formatted);
                rxDirty = true;
            }
        }
    });


    std::cout << "Opened " << PORT << " @ " << BAUD << "\n"
              << "Keys: W forward | S back | A left(in-place) | D right(in-place)\n"
              << "      SPACE stop | Q quit | +/- speed\n";

    int speed = 40;
    int turnSpeed = 35;

    MotorCmd cmd{};
    MotorCmd lastSent{};

    while (running.load()) {
        int k = read_key_nonblock();
        if (k >= 0) {
            char c = (char)k;
            if (c >= 'A' && c <= 'Z') c = (char)(c - 'A' + 'a');

            if (c == 'q') {
                cmd = {0, 0, 1, 1};
                write_motor(fd, cmd);
                running.store(false);
                break;
            } else if (c == ' ') {
                cmd = {0, 0, 1, 1};
            } else if (c == '+') {
                speed = clampi(speed + 5, 0, 100);
                std::cout << "\rSpeed=" << speed << "%   " << std::flush;
            } else if (c == '-') {
                speed = clampi(speed - 5, 0, 100);
                std::cout << "\rSpeed=" << speed << "%   " << std::flush;
            } else if (c == 'w') {
                cmd = {speed, speed, 1, 1};
            } else if (c == 's') {
                cmd = {speed, speed, 0, 0};
            } else if (c == 'a') {
                cmd = {turnSpeed, turnSpeed, 1, 0}; // right fwd, left back
            } else if (c == 'd') {
                cmd = {turnSpeed, turnSpeed, 0, 1}; // right back, left fwd
            }
        }

        // Send only on change
        if (cmd.pwmR != lastSent.pwmR || cmd.pwmL != lastSent.pwmL ||
            cmd.dirR != lastSent.dirR || cmd.dirL != lastSent.dirL) {
            if (write_motor(fd, cmd)) {
                lastSent = cmd;
            }
        }
        auto lastPrint = std::chrono::steady_clock::now();
        std::string lastRxPrinted;
        // 1 Hz display (same line, no newline)
        auto now = std::chrono::steady_clock::now();
        if (now - lastPrint >= std::chrono::seconds(1)) {
            std::string rxCopy;
            bool haveNew = false;
            {
                std::lock_guard<std::mutex> lk(rxMutex);
                if (rxDirty) {
                    rxCopy = latestRx;
                    rxDirty = false;
                    haveNew = true;
                }
            }
            if (haveNew) lastRxPrinted = std::move(rxCopy);

            // Build one status line: TX + RX (pad to clear leftovers)
            std::string line = "TX: " + std::to_string(lastSent.pwmR) + "," +
                            std::to_string(lastSent.pwmL) + "," +
                            std::to_string(lastSent.dirR) + "," +
                            std::to_string(lastSent.dirL) +
                            " | " + lastRxPrinted;

            // Clear line by padding/truncation
            if (line.size() < 120) line.append(120 - line.size(), ' ');
            else line.resize(120);

            std::cout << "\r" << line << std::flush;
            lastPrint = now;
        }

        usleep(10 * 1000);
    }

    if (rxThread.joinable()) rxThread.join();
    close(fd);
    std::cout << "\nExit.\n";


    return 0;
}