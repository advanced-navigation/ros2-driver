/****************************************************************/
/*                                                              */
/*                       Advanced Navigation                    */
/*         				       Logger			  			    */
/*          Copyright 2023, Advanced Navigation Pty Ltd         */
/*                                                              */
/****************************************************************/
/*
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef ADNAV_LOGGER_H_
#define ADNAV_LOGGER_H_

#include <string>
#include <iomanip>
#include <sstream>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <ctime>
#include <memory>
#include <stdexcept>

namespace adnav{

/**
 * @brief Wrapper class for a std::ofstream. Allows for creation of a logging
 * file at a specified location, with a prefix and file type suffix specifiable
 * by external calls. 
 * 
 * This class will also handle exceptions and provide warnings to std::cerr 
 * and std::cout. 
*/
class Logger : public std::ofstream{
    public:
        /**
         * @brief Default constructor. 
        */
        Logger():write_counter_(0),flush_after_(0){}

        /**
         * @brief Fully parameterised constructor that will open the file using the provided information. 
        */
        Logger(const std::string& prefix, const std::string& file_type, int interval, const std::string& path = "",
            std::ios_base::openmode mode = std::ofstream::out):write_counter_(0),flush_after_(interval){
            openFile(prefix, file_type, path, mode);
        }

        ~Logger(){}       

        /**
         * @brief Function to open an output Logging file stream at a given file path.
         * It will name the file <prefix>_YY-MM-DD_HH-MM-SS<filetype>
         *
         * @param prefix The prefix in the filename
         * @param file_type The filetype suffix for the file. 
         * @param path The file path to open the file in. If empty will be placed in
         * the current wd. 
         * @param mode The opening mode for the filestream. out is always specified. 
         * Bitwise or'ing multiple of these will set the behaviour of the stream. Even 
         * though Logger is a ofstream child, its internal filebuf can be set to also
         * support input operations. 
         * see https://cplusplus.com/reference/ios/ios_base/openmode/ for more 
         * information on the opening modes. 
        */
        void openFile(const std::string& prefix, const std::string& file_type,
            const std::string& path = "", std::ofstream::openmode mode = std::ofstream::out){
            // Set the filename of the log file
            setFilename(prefix, file_type, path);

            // Check to see if there the file is open already. 
            if(this->is_open()){
                throw std::runtime_error(std::string("File already open can't open new file:") + filename_);
            }
            
            // Open a the new file. 
            this->open(filename_, mode);

            if( (this->rdstate() & std::ofstream::failbit) != 0){
                std::cerr << "Error opening log file: " << filename_ << std::endl;
            }

        }
        
        /**
         * @brief Function to write to the file buffer and flush if the number of writes has
         * reached the set amount. 
         * 
         * @param s The buffer to write
         * @param n The size of the buffer to write. 
        */
        void writeAndIncrement(const char* s, std::streamsize n){
            // Check if the stream has failed. 
            if( (this->rdstate() & std::ofstream::failbit) != 0){
                return;
            }

            // Write to the file buffer
            this->write(s, n);

            // Check to ensure that no writing error occurred
            if( (this->rdstate() & std::ofstream::badbit) != 0){
                std::cerr << "Error writing to " << filename_ << "Output buffer." << std::endl;
                return;
            }

            if(write_counter_++ >= flush_after_){
                this->flush();
                write_counter_ = 0;
                if((this->rdstate() & std::ofstream::badbit) != 0){
                    std::cerr << "Error flushing " << filename_ << "'s buffer." << std::endl;
                }
            }
        }

        /**
         * @brief Method to set the interval between flushes of the writing buffer. 
         * The buffer by default is zero and will flush the buffer after every write. 
        */
        void setFlushInterval(int interval){flush_after_ = interval;}

        /**
         * @brief Function to close the file and output to std::cout the location of the file. 
         * If an error occurs it will output a descriptive failure to std::cerr.
        */
        void closeFile(){
            this->close();

            if((this->rdstate() & std::ifstream::failbit) != 0) {
                std::cerr << "ERROR: Failure closing log file: " << filename_ << std::endl;
            }
            else {
                std::cout << "Closed Log file: " << filename_ << std::endl;
            }
        }

    private:
        time_t time_;
        struct tm * timeInfo_;
        std::string filename_;
        int write_counter_;
        int flush_after_;

        /**
         * @brief Function to take the individual elements of the filename and turn them into the
         * private filename member. 
         * This places it into the format <path><prefix>_YY-MM-DD_HH-MM-SS<file_type>
        */
        void setFilename(const std::string& prefix, const std::string& file_type,
            const std::string& path = ""){
            // make a stringstream 
            std::stringstream ss;
            time(&time_);
            timeInfo_ = localtime(&time_);
            std::string local_path = path;
            std::string local_prefix = prefix;


            // If the string path starts with a ~ shortcut for home, adjust it.
            if(path.at(0) == '~'){
                local_path.replace(0, 1, getenv("HOME"));
            }

            // If the last character of the prefix is a '_' remove it.
            if(prefix.back() == '_') local_prefix.pop_back();

            ss << local_path << local_prefix << std::put_time(timeInfo_, "_%y-%m-%d_%H-%M-%S") << file_type;

            filename_ = ss.str();
        }
};


}// namespace adnav

#endif //ADNAV_LOGGER_H_