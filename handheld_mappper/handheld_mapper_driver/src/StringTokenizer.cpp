//---------- StringTokenizer.cpp ---------------

/**
 * 생성자
 */
StringTokenizer::StringTokenizer(const string& pStr, const string& pDelimeter)
{
   if ((pStr.length() == 0) || (pDelimeter.length() == 0)) return;
   iStr = pStr;
   iDelimeter     = pDelimeter;
  // 연속 구분자 제거
   unsigned int tPosition = 0;
   while(true)
   {
      if ((tPosition = iStr.find(iDelimeter,tPosition)) != string::npos)
      {
         tPosition += iDelimeter.length();
         while(iStr.find(iDelimeter,tPosition) == tPosition)
         {
            iStr.erase(tPosition,iDelimeter.length());
         }
      }
      else
       break;
   }
   // 맨앞에 나오는 구분자 제거
   if (iStr.find(iDelimeter,0) == 0)
   {
      iStr.erase(0,iDelimeter.length());
   }
   // 맨뒤에 나오는 구분자 제거
   tPosition = 0;
   if ((tPosition = iStr.rfind(iDelimeter)) != string::npos)
   {
      if (tPosition != (iStr.length() - iDelimeter.length())) return;
      iStr.erase(iStr.length() - iDelimeter.length(),iDelimeter.length());
   }
}
/**
 * 총 token 갯수
 */
int StringTokenizer::countTokens()
{
   unsigned int tPrevPosition = 0;
   int tNoOfTokens        = 0;
   if (iStr.length() > 0)
   {
      tNoOfTokens = 0;
      unsigned int tPosition = 0;
      while(true)
      {
         if ((tPosition = iStr.find(iDelimeter,tPosition)) != string::npos)
         {
            tNoOfTokens++;
            tPrevPosition  = tPosition;
            tPosition += iDelimeter.length();
         }
         else
          break;
      }
      return ++tNoOfTokens;
   }
   else
   {
      return 0;
   }
}
/**
 * 다음 token 존재 여부 리턴
 */
bool StringTokenizer::hasMoreTokens()
{
   return (iStr.length() > 0);
}
/**
 * 다음 token 리턴
 */
string StringTokenizer::nextToken()
{
   if (iStr.length() == 0)
     return "";
   string  tReturnStr = "";
   unsigned int tPosition     = iStr.find(iDelimeter,0);
   if (tPosition != string::npos)
   {
      tReturnStr   = iStr.substr(0,tPosition);
      iStr = iStr.substr(tPosition+iDelimeter.length(),iStr.length()-tPosition);
   }
   else
   {
      tReturnStr   = iStr.substr(0,iStr.length());
      iStr = "";
   }
   return tReturnStr;
}
/**
 * 다음 token 리턴
 */
string StringTokenizer::nextToken(const string& iDelimeteriter)
{
   if (iStr.length() == 0)
     return "";
   string  tReturnStr = "";
   unsigned int pos     = iStr.find(iDelimeteriter,0);
   if (pos != string::npos)
   {
      tReturnStr   = iStr.substr(0,pos);
      iStr = iStr.substr(pos + iDelimeteriter.length(),iStr.length() - pos);
   }
   else
   {
      tReturnStr   = iStr.substr(0,iStr.length());
      iStr = "";
   }
   return tReturnStr;
} 
