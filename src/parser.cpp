#include <iostream>
#include "parser.hpp"

void openFile(std::ifstream &file, std::string filePath)
{
    file.open(filePath);
    if (file.is_open())
    {
        std::cout << "File opened successfully" << std::endl;
    }
    else
    {
        std::cerr << "File open failed\n"
                  << std::endl;
        exit(-1);
    }
}

// this method parse an instance and create a matrix of int from the 7th line
std::vector<std::vector<int> > processFile(std::ifstream &file)
{
    // data format:
    // 100000000 26 82 65
    // 66 100000000 56 39
    // 43 57 100000000 16
    std::string line;
    std::vector<std::vector<int> > matrix;
    std::vector<int> row;
    char const delimiter = ' ';

    // skip the 7th first lines
    for (int i = 0; i < 7; i++)
    {
        std::getline(file, line);
    }

    // parse the rest of the file
    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        int value;
        while (ss >> value)
        {
            row.push_back(value);
            if (ss.peek() == delimiter)
            {
                ss.ignore();
            }
        }
        matrix.push_back(row);
        row.clear();
    }
    return matrix;
}

std::vector<std::vector<int> > parse(std::string filePath)
{
    std::ifstream file;
    openFile(file, filePath);
    std::vector<std::vector<int> > matrix = processFile(file);
    file.close();
    return matrix;
}
