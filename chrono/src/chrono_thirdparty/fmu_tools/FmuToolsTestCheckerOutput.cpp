#include <iostream>
#include <fstream>
#include <regex>
#include <string>

int main() {
    std::cout << "Opening fmuChecker log: " << FMUCHECKER_LOG << std::endl;
    std::cout << "-> status: ";
    std::ifstream file(FMUCHECKER_LOG);
    if (!file.is_open()) {
        std::cerr << "ERROR: cannot open file: " << FMUCHECKER_LOG << std::endl;
        return 1;
    }
    std::cout << " SUCCESS" << std::endl;

    std::regex errorRegex(R"(\[ERROR\](.*))");
    std::regex warningRegex(R"(\[WARNING\](.*))");
    std::string line;

    std::cout << "Parsing fmuChecker log..." << std::endl;
    unsigned int errorCount = 0;
    unsigned int warningCount = 0;
    while (std::getline(file, line)) {
        std::smatch match;
        if (std::regex_search(line, match, errorRegex)) {
            std::cout << "ERROR: " << match[1] << std::endl;
            errorCount++;
        }
        else if (std::regex_search(line, match, warningRegex)) {
            std::cout << "WARNING: " << match[1] << std::endl;
            warningCount++;
        }
    }

    file.close();
    std::cout << "Parsing completed with:" << std::endl;
    std::cout << "- errors: " << errorCount << std::endl;
    std::cout << "- warnings: " << warningCount << std::endl;

    return errorCount>0;
}