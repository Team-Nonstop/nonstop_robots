#include <vector>

void
split( std::vector<std::string> & theStringVector,  /* Altered/returned value */
       const  std::string  & theString,
       const  std::string  & theDelimiter)
{
   // UASSERT( theDelimiter.size(), >, 0); // My own ASSERT macro.

    size_t  start = 0, end = 0;

    while ( end != std::string::npos)
    {
        end = theString.find( theDelimiter, start);

        // If at end, use length=maxLength.  Else use length=end-start.
        theStringVector.push_back( theString.substr( start,
                       (end == std::string::npos) ? std::string::npos : end - start));

        // If at end, use start=maxSize.  Else use start=end+delimiter.
        start = (   ( end > (std::string::npos - theDelimiter.size()) )
                  ?  std::string::npos  :  end + theDelimiter.size());
    }
}


inline std::string trim_left(const std::string& str) 
{ 
    int n = str.find_first_not_of(IMU_RESP_BEGIN); 
    return n == std::string::npos ? str : str.substr(n, str.length()); 
} 

inline std::string trim_right(const std::string& str) 
{ 
    int n = str.find_last_not_of(IMU_RESP_END); 
    return n == std::string::npos ? str : str.substr(0, n + 1); 
} 
