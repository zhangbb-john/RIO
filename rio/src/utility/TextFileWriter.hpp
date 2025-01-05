#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <functional> // For function pointers like std::endl

class TextFileWriter {
private:
    std::string filePath; // Path to the text file
    std::ofstream file;   // Output file stream
    bool isInitialized;   // Flag to track initialization
public:
    // Constructor to initialize file path and open the file
    TextFileWriter(const std::string& path, const std::string& fileName) {
        filePath = path + "/" + fileName;
        file.open(filePath, std::ios::app); // Open file in append mode
        if (!file.is_open()) {
            throw std::ios_base::failure("Error: Could not open file at " + filePath);
        }
        isInitialized = false;
    }
    TextFileWriter() : isInitialized(false) {}

    
    // Destructor to ensure the file is properly closed
    ~TextFileWriter() {
        if (file.is_open()) {
            file.close();
        }
    }

    void initialize(const std::string& path, const std::string& fileName) {
        filePath = path + "/" + fileName;
        file.open(filePath, std::ios::app); // Open file in append mode
        if (!file.is_open()) {
            throw std::ios_base::failure("Error: Could not open file at " + filePath);
        }
        isInitialized = true;
    }

    // writeToFile implementation
    void writeToFile(const std::string& content) {
        if (file.is_open()) {
            file << content << std::endl;
        } else {
            throw std::ios_base::failure("Error: File is not open for writing.");
        }
    }
    // Overload the << operator for generic content
    template <typename T>
    TextFileWriter& operator<<(const T& content) {
        if (file.is_open()) {
            file << std::fixed << std::setprecision(3) << content;
        } else {
            throw std::ios_base::failure("Error: File is not open for writing.");
        }
        return *this;
    }

    // Overload the << operator for manipulators (like std::endl)
    TextFileWriter& operator<<(std::ostream& (*manip)(std::ostream&)) {
        if (file.is_open()) {
            manip(file); // Apply the manipulator to the underlying std::ofstream
        } else {
            throw std::ios_base::failure("Error: File is not open for writing.");
        }
        return *this;
    }
};

// int main() {
//     try {
//         // Example usage
//         TextFileWriter writer("C:/example/path", "output.txt");

//         // Write to the file using the << operator
//         writer << "This is the first line." << std::endl
//                << "This is the second line." << std::endl
//                << "This is written using << operator with std::endl support." << std::endl;

//         std::cout << "Done writing to the file!" << std::endl;
//     } catch (const std::exception& ex) {
//         std::cerr << ex.what() << std::endl;
//     }

//     return 0;
// }
