#define _CRT_SECURE_NO_WARNINGS 

// ****************************************************************************
// File: BingTranslate.cpp
// Desc: Bing/Microsoft Translator support
//
// ****************************************************************************
#include "stdafx.h"
#include "BingTranslate.h"
#pragma comment(lib, "Wininet.lib")

#define SIZESTR(x) (sizeof(x) - 1)

using namespace BingTranslate;


// Minimum time in seconds to request a new token
const float AUTH_TIME_MARGIN = 20.0f;

// Connection time-out in ms
const DWORD CONNECT_TIMEOUT = (30 * 1000);

// Max allowed translation string size (MS limit)
const UINT MAX_INPUT_TEXT = 10000;

// Max mutli-translation count (MS limit)
const UINT MAX_TRANSLATE_COUNT = 2000;


// Encode string in HTML forms "application/x-www-form-urlencoded" spec
// AFAIK there isn't a working Win API function for it
// The output buffer size should be ((iInputSize + 1) * 3 + 1)  to allow for the
// worst case of every character needing encoding and +1 for a terminator.
// Returns pszOut or NULL on buffer space overflow.
//
// Refs:
// http://www.w3.org/TR/html401/interact/forms.html#h-17.13.4.1
// http://en.wikipedia.org/wiki/Percent-encoding
static LPSTR FormURLEncode(__in_z LPCSTR pszSrc, int iInputSize, __out_z LPSTR pszOut, int iOutSize)
{
	// RFC 3986 section 2.3, reserved, unreserved, and space character routing
	__declspec(align(16)) static const BYTE aCharRoute[256] =
	{
		3,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		1,2,2,2,2,2,2,2,2,2,2,2,2,0,0,2,0,0,0,0,0,0,0,0,0,0,2,2,2,2,2,2,
		2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,2,2,2,0,
		2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,2,2,0,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
	};
	// To spec A-F can be upper or lower case but the later seems more standard/popular
	static const BYTE aN2Hex[] = {"0123456789abcdef"};

	PBYTE pSrc = (PBYTE) pszSrc;
	PBYTE pDst = (PBYTE) pszOut;

	while(iInputSize-- && (iOutSize > 0))
	{
		BYTE c = *pSrc++;
		switch(aCharRoute[c])
		{
			// Pass along unreserved characters
			case 0:
			*pDst++ = c, iOutSize--;
			break;

			// Encode space character
			case 1:
			*pDst++ = '+', iOutSize--;
			break;

			// Percent encode reserved characters
			case 2:
			{
				pDst[0] = '%';
				pDst[1] = aN2Hex[c >> 4];
				pDst[2] = aN2Hex[c & 0xF];
				pDst +=3, iOutSize -= 3;
			}
			break;

			// Bail on terminator
			case 3:
			iInputSize = 0;
			break;
		};
	};

	if(iOutSize >= 0)
	{
		*pDst = 0;
		return(pszOut);
	}
	else
		return(NULL);
}

// Get WinINet error code string
static std::string &GetWininetErrorString(DWORD dwError, std::string &rcOutput)
{
	char szBuffer[4096];
	if(FormatMessage((FORMAT_MESSAGE_IGNORE_INSERTS | FORMAT_MESSAGE_FROM_HMODULE),
	GetModuleHandle("wininet.dll"), dwError, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
	szBuffer, (sizeof(szBuffer) - 1), NULL))
		rcOutput = szBuffer;
	else
		rcOutput = "Unknown WinINet error";
	return(rcOutput);
}

// Get a time stamp in 1ms units (w/typical granularity of 10ms)
static UINT64 GetTimeStamp64()
{
	FILETIME FileTime;
	GetSystemTimeAsFileTime(&FileTime);
	return(*((PUINT64) &FileTime) / 10000);
}


// ------------------------------------------------------------------------------------------------

// Authentication handler to obtain a ~10min temporal access token
// Refs:
// https://datamarket.azure.com/developer/applications/
// http://msdn.microsoft.com/en-us/library/hh454950.aspx
// http://msdn.microsoft.com/en-us/library/windowsazure/hh781551.aspx
// http://wiki.oauth.net/w/page/25236487/OAuth%202

class BingTranslate::Authenticate
{
public:
	Authenticate(__in_z LPCSTR pszEncodedID, __in_z LPCSTR pszEncodedSecret, __out RESULT &rResult);
	~Authenticate();

	// Get access token prepended with the "Bearer" keyword and suffixed with HTTP header line feed
	LPCSTR GetToken() { return((LPCSTR) m_AccessToken.c_str()); }

	// Return remaining valid token time in fractional seconds
	float GetRemainingTime()
	{
		float d = (float) ((double) (m_ExpirationTime - GetTimeStamp64()) * (double) (1.0 / 1000.0));
		return((d >= 0) ? d : 0);
	}

private:
	HINTERNET   m_hSession, m_hConnect, m_hRequest;
	std::string m_AccessToken;
	UINT64      m_ExpirationTime;
};

BingTranslate::Authenticate::Authenticate(__in_z LPCSTR pszEncodedID, __in_z LPCSTR pszEncodedSecret, __out RESULT &rResult) : m_hSession(NULL), m_hConnect(NULL), m_hRequest(NULL), m_ExpirationTime(-1)
{
	rResult.Clear();
	#define SET_ERROR(str)	\
	{	\
		std::string cError; \
		GetWininetErrorString(GetLastError(), cError);	\
		rResult.Info = str;	\
		rResult.Info += " error: " + cError;	\
	}

	// Create form data
	char szFormData[4096]; szFormData[SIZESTR(szFormData)] = 0;
	_snprintf(szFormData, SIZESTR(szFormData),
		"grant_type=client_credentials"
		"&client_id=%s"
		"&client_secret=%s"
		"&scope=http://api.microsofttranslator.com",
		pszEncodedID, pszEncodedSecret);

	// Create WinINet session
	if(m_hSession = InternetOpen("", INTERNET_OPEN_TYPE_PRECONFIG, NULL, NULL, 0))
	{
		DWORD dwOption = CONNECT_TIMEOUT;
		InternetSetOption(m_hSession, INTERNET_OPTION_CONNECT_TIMEOUT, &dwOption, sizeof(dwOption));
		dwOption = CONNECT_TIMEOUT;
		InternetSetOption(m_hSession, INTERNET_OPTION_DATA_SEND_TIMEOUT, &dwOption, sizeof(dwOption));
		dwOption = CONNECT_TIMEOUT;
		InternetSetOption(m_hSession, INTERNET_OPTION_DATA_RECEIVE_TIMEOUT, &dwOption, sizeof(dwOption));
		dwOption = CONNECT_TIMEOUT;
		InternetSetOption(m_hSession, INTERNET_OPTION_RECEIVE_TIMEOUT, &dwOption, sizeof(dwOption));

		// Set HTTPS connect URL for the MS authentication site
		if(m_hConnect = InternetConnect(m_hSession, "datamarket.accesscontrol.windows.net", INTERNET_DEFAULT_HTTPS_PORT, NULL, NULL, INTERNET_SERVICE_HTTP, INTERNET_FLAG_RESYNCHRONIZE, 1))
		{
			// Create our POST request to URL
			const UINT RQFLAGS =
			(
				INTERNET_FLAG_SECURE |
				INTERNET_FLAG_KEEP_CONNECTION |
				INTERNET_FLAG_PRAGMA_NOCACHE |
				INTERNET_FLAG_NO_CACHE_WRITE |
				INTERNET_FLAG_RELOAD |
				INTERNET_FLAG_NO_COOKIES |
				INTERNET_FLAG_NO_UI
			);

			LPCSTR pszAcceptedTypes[] = {"application/json", NULL};
			if(m_hRequest = HttpOpenRequest(m_hConnect, "POST", "v2/OAuth2-13/", NULL, NULL, pszAcceptedTypes, RQFLAGS, 1))
			{
				// Create header (typically ~164 chars in length)
				char szHeaders[256]; szHeaders[SIZESTR(szHeaders)] = 0;
				_snprintf(szHeaders, SIZESTR(szHeaders),
					"Content-Type: application/x-www-form-urlencoded\r\n"
					"Host: datamarket.accesscontrol.windows.net\r\n"
					"Content-Length: %d\r\n"
					"Expect: 100-continue\r\n"
					"Connection: keep-alive\r\n"
					"\r\n\r\n",
					strlen(szFormData)
					);

				// Do POST request with our form data
				if(HttpSendRequest(m_hRequest, szHeaders,strlen(szHeaders), szFormData,strlen(szFormData)))
				{
					// Get JSON formated response (typically ~600 chars in length)
					char szResponse[4096];
					DWORD dwBytesRead = 0;
					if(InternetReadFile(m_hRequest, szResponse,SIZESTR(szResponse), &dwBytesRead) && (dwBytesRead > 0))
					{
						szResponse[dwBytesRead] = 0;

						// Parse the data
						// Yes we could use a real parser like "jsoncpp" but this simple parsing is adequate

						// Access token
						const char ACTSTART[] = {"\"access_token\":\""};
						const char ENDTAG[]   = {"\","};
						if(LPSTR pszACToken = strstr(szResponse, ACTSTART))
						{
							pszACToken += SIZESTR(ACTSTART);
							if(LPSTR pszEnd = strstr(pszACToken + 32, ENDTAG))
							{
								*pszEnd = 0;
								_ASSERT(strlen(pszACToken) > 32);

								// Store it w/prefixed usage keyword, and HTTP line feeds at the end
								m_AccessToken = "Bearer ";
								m_AccessToken += pszACToken;
								m_AccessToken += "\r\n";

								// Token expiration seconds
								const char EXINSTART[] = {"\"expires_in\":\""};
								if(LPSTR pszExpiresStart = strstr(pszEnd + SIZESTR(ENDTAG), EXINSTART))
								{
									pszExpiresStart += SIZESTR(EXINSTART);
									if(LPSTR pszEnd = strchr(pszExpiresStart, '"'))
									{
										*pszEnd = 0;

										// Will typically be ~600 seconds (ten minutes)
										UINT uExpireSeconds = atoi(pszExpiresStart);
										_ASSERT(uExpireSeconds > 500);
										m_ExpirationTime = (GetTimeStamp64() + ((uExpireSeconds - 1) * 1000));
										rResult.SetSuccess();
									}
								}
							}
						}

						// Parse failed, most likely the authentication server responded with fail result string
						if(!rResult.IsSuccess())
						{
							char szHeader[1024]; szHeader[0] = szHeader[SIZESTR(szHeader)] = 0;
							DWORD dwHeaderSize  = SIZESTR(szHeader);
							if(HttpQueryInfo(m_hRequest, HTTP_QUERY_RAW_HEADERS_CRLF, szHeader,&dwHeaderSize, NULL))
							{
								szHeader[dwHeaderSize] = 0;
								//printf("Header1: \"%s\"\n", szHeader);
							}

							rResult.Info = "Authentication error!\nHeader:\n\"";
							rResult.Info += szHeader;
							rResult.Info += "\"\nResponse data:\n\"";
							rResult.Info += szResponse;
							rResult.Info += "\".";
						}
					}
					else
						SET_ERROR("InternetReadFile()");
				}
				else
					SET_ERROR("HttpSendRequest()");
			}
			else
				SET_ERROR("HttpOpenRequest()");
		}
		else
			SET_ERROR("InternetConnect()");
	}
	else
		SET_ERROR("InternetOpen()");

	#undef SET_ERROR
}

BingTranslate::Authenticate::~Authenticate()
{
	if(m_hRequest) InternetCloseHandle(m_hRequest);
	if(m_hConnect) InternetCloseHandle(m_hConnect);
	if(m_hSession) InternetCloseHandle(m_hSession);
	m_hRequest = m_hConnect = m_hSession = NULL;
}


// ------------------------------------------------------------------------------------------------

// Input: Raw ID and Secret/key strings
Translator::Translator(__in_z LPCSTR pszID, __in_z LPCSTR pszSecret, __out RESULT &rResult) : m_pcAuth(NULL), m_hSession(NULL), m_pszBuffer(NULL)
{
	rResult.Clear();
	#define SET_ERROR(str)	\
	{	\
		std::string cError; \
		GetWininetErrorString(GetLastError(), cError);	\
		rResult.Info = str;	\
		rResult.Info += " error: " + cError;	\
	}

	if(pszID && pszSecret)
	{
		// Encode args for transmission
		char szBuffer[4096] = {0};
		if(FormURLEncode(pszID,strlen(pszID), szBuffer,(sizeof(szBuffer) - 7)))
		{
			m_EncodedID = szBuffer;

			if(FormURLEncode(pszSecret,strlen(pszSecret), szBuffer, (sizeof(szBuffer) - 4)))
			{
				m_EncodedSecret = szBuffer;

				// Create WinINet session
				if(m_hSession = InternetOpen("", INTERNET_OPEN_TYPE_PRECONFIG, NULL, NULL, 0))
				{
					DWORD dwOption = CONNECT_TIMEOUT;
					InternetSetOption(m_hSession, INTERNET_OPTION_CONNECT_TIMEOUT, &dwOption, sizeof(dwOption));
					dwOption = CONNECT_TIMEOUT;
					InternetSetOption(m_hSession, INTERNET_OPTION_DATA_SEND_TIMEOUT, &dwOption, sizeof(dwOption));
					dwOption = CONNECT_TIMEOUT;
					InternetSetOption(m_hSession, INTERNET_OPTION_DATA_RECEIVE_TIMEOUT, &dwOption, sizeof(dwOption));
					dwOption = CONNECT_TIMEOUT;
					InternetSetOption(m_hSession, INTERNET_OPTION_RECEIVE_TIMEOUT, &dwOption, sizeof(dwOption));
				
					if(m_pszBuffer = (LPSTR) _aligned_malloc(m_iBufferSize = 2048, 16))
					{
						rResult.SetSuccess();
					}
					else
						rResult.Info = "Text buffer alloc failed";					
				}
				else
					SET_ERROR("InternetOpen()");
			}
			else
				rResult.Info = "\"Secret\" string buffer overflow";
		}
		else
			rResult.Info = "ID string buffer overflow";
	}
	else
		rResult.Info = "Missing argument";

	#undef SET_ERROR
}

Translator::~Translator()
{
	_aligned_free(m_pszBuffer);
	m_pszBuffer = NULL;

	if(m_pcAuth) delete m_pcAuth;		
	m_pcAuth = NULL;	
	
	if(m_hSession) InternetCloseHandle(m_hSession);
	m_hSession = NULL;
}

// Create, or recreate the temporal authentication token as needed
BOOL BingTranslate::Translator::UpdateAuthentication(__out RESULT &rResult)
{
	rResult.Clear();

	if(!m_pcAuth || (m_pcAuth && (m_pcAuth->GetRemainingTime() < AUTH_TIME_MARGIN)))
	{
		// Kill previous
		if(m_pcAuth)
		{
			delete m_pcAuth;
			m_pcAuth = NULL;
		}

		// Instance and get
		if(!(m_pcAuth = new Authenticate(m_EncodedID.c_str(), m_EncodedSecret.c_str(), rResult)))
			rResult.Info = "Failed to allocate Authenticate instance";
	}
	else
		rResult.SetSuccess();

	return(rResult.IsSuccess());
}


// Returns TRUE if can connect via internet to the Bing Translator
// On success returns connection, or error on failure
BOOL BingTranslate::Translator::CanConnect(__out RESULT &rOutput)
{
	rOutput.Clear();
	#define SET_ERROR(str)	\
	{	\
		std::string cError; \
		GetWininetErrorString(GetLastError(), cError);	\
		rOutput.Info = str;	\
		rOutput.Info += cError;	\
	}

	// Basic connection status?
	DWORD dwFlags = 0;
	char szBuffer[2048] = {0};
	if(InternetGetConnectedStateEx(&dwFlags, szBuffer,SIZESTR(szBuffer), 0))
	{
		if(!(dwFlags & INTERNET_CONNECTION_OFFLINE))
		{
			// Note: Apparently never sets last error on failure. Appears to be always 0.
			// TODO: Or use the authentication server https://datamarket.azure.com?
			if(InternetCheckConnection("http://api.microsofttranslator.com", FLAG_ICC_FORCE_CONNECTION, 0))
			{
				rOutput.Info = szBuffer;
				rOutput.SetSuccess();
			}
			else
				SET_ERROR("Failed to connect: ");
		}
		else
			rOutput.Info = "Connection offline status.";
	}
	else
		SET_ERROR("Bad connnection state: ");

	#undef SET_ERROR
	return(rOutput.IsSuccess());
}


// Translate an UTF-8 text input string
// Inputs:
// pszText     - Input UTF-8 text string
// iTextLen    - Size of input bytes/chars, or -1 for automatic.
// pszFromLang - Input two letter language code, or NULL for automatic detection.
// pszToLang   - The translate "to" language code, or NULL to default to English.
//
// Output:
// rOutput.Result - Translated UTF-8 string, or error info on failure
//
// Refs:
// "Translator Language Codes" http://msdn.microsoft.com/en-us/library/hh456380.aspx
// "Using the HTTP Interface" http://msdn.microsoft.com/en-us/library/ff512387.aspx
// "Translate Method" http://msdn.microsoft.com/en-us/library/ff512421.aspx
// "REpresentational State Transfer (REST) " http://en.wikipedia.org/wiki/Representational_state_transfer
//
BOOL Translator::Translate(__in LPCSTR pszInput, int iInputLen, __in_opt LPCSTR pszFromLang, __in LPCSTR pszToLang, __out RESULT &rOutput)
{
	#define SET_ERROR(str) { rOutput.Info = str; }
	#define SET_ERROR2(str)	\
	{	\
		std::string cError; \
		GetWininetErrorString(GetLastError(), cError);	\
		rOutput.Info = str;	\
		rOutput.Info += cError;	\
	}
	rOutput.Clear();

	if(pszInput)
	{
		if(iInputLen == -1) iInputLen = strlen(pszInput);
		if(iInputLen > 0)
		{
			if(iInputLen <= MAX_INPUT_TEXT)
			{
				// Create, or expire check existing temporal authentication token
				if(UpdateAuthentication(rOutput))
				{
					rOutput.SetFail();

					// Allow for the worst case where every input char is encoded
					int iMaxSize = ((iInputLen * SIZESTR("%FF")) + 8);

					// Expand text buffer as needed
					if(iMaxSize > m_iBufferSize)
					{
						_aligned_free(m_pszBuffer);
						m_pszBuffer = (LPSTR) _aligned_malloc(m_iBufferSize = iMaxSize, 16);
					}

					if(m_pszBuffer)
					{
						// Encode input string
						if(FormURLEncode(pszInput,iInputLen, m_pszBuffer,(iMaxSize - 4)))
						{
							// Request translate via URL
							// Create GET URL line
							std::string FormURL("http://api.microsofttranslator.com/v2/Http.svc/Translate?text=");
							FormURL += m_pszBuffer;
							// Optional "from" language code
							if(pszFromLang)
							{
								FormURL += "&from=";
								FormURL += pszFromLang;
							}
							if(pszToLang)
							{
								FormURL += "&to=";
								FormURL += pszToLang;
							}
							else
								// Default to English
								FormURL += "&to=en";

							// Make header
							// Appears "Authorization:" must come first, don't change the order
							std::string Headers("Authorization: ");
							Headers += m_pcAuth->GetToken();
							Headers += "Host: api.microsofttranslator.com\r\n"
							"Connection: Keep-Alive\r\n"
							"\r\n\r\n";

							const UINT OPENFLAGS =
							(
								INTERNET_FLAG_KEEP_CONNECTION |
								INTERNET_FLAG_NO_CACHE_WRITE |
								INTERNET_FLAG_NO_COOKIES |
								INTERNET_FLAG_NO_UI |
								INTERNET_FLAG_IGNORE_REDIRECT_TO_HTTPS
							);

							if(HINTERNET hTransaction = InternetOpenUrl(m_hSession, FormURL.c_str(), Headers.c_str(),Headers.size(), OPENFLAGS, 2))
							{
								// Expand buffer as needed for response data
								DWORD dwBytesAvailable = 0;
								if(InternetQueryDataAvailable(hTransaction, &dwBytesAvailable, 0,0))
								{
									if(dwBytesAvailable > (UINT) m_iBufferSize)
									{
										_aligned_free(m_pszBuffer);
										m_pszBuffer = (LPSTR) _aligned_malloc(m_iBufferSize = (dwBytesAvailable + max(16, (dwBytesAvailable >> 2))), 16);
									}
									if(m_pszBuffer)
									{
										DWORD dwBytesRead = 0;
										if(InternetReadFile(hTransaction, m_pszBuffer,(m_iBufferSize - 1), &dwBytesRead) && (dwBytesRead > 0))
										{
											_ASSERT(dwBytesRead == dwBytesAvailable);
											m_pszBuffer[dwBytesRead] = 0;

											// Parse out XML formated string
											if(LPSTR pszString = strstr(m_pszBuffer, "<string xmlns=\"http://schemas.microsoft.com/2003/10/Serialization/\">"))
											{
												pszString += SIZESTR("<string xmlns=\"http://schemas.microsoft.com/2003/10/Serialization/\">");
												if(LPSTR pszEnd = strstr(pszString, "</string>"))
												{
													*pszEnd = 0;
													rOutput.Info = pszString;
													rOutput.SetSuccess();
												}
											}

											if(!rOutput.IsSuccess())
											{
												char szHeader[1024]; szHeader[0] = szHeader[SIZESTR(szHeader)] = 0;
												DWORD dwHeaderSize  = SIZESTR(szHeader);
												if(HttpQueryInfo(hTransaction, HTTP_QUERY_RAW_HEADERS_CRLF, szHeader,&dwHeaderSize, NULL))
												{
													szHeader[dwHeaderSize] = 0;
													//printf("Header2: \"%s\"\n", szHeader);
												}

												rOutput.Info = "Translate response parse failure!\nHeader:\n\"";
												rOutput.Info += szHeader;
												rOutput.Info += "\"\nResponse data:\n\"";
												rOutput.Info += m_pszBuffer;
												rOutput.Info += "\".";
											}
										}
										else
											SET_ERROR2("InternetReadFile() error:");
									}
									else
										SET_ERROR("Failed to grow text buffer");
								}
								else
									SET_ERROR2("InternetQueryDataAvailable() error: ");

								InternetCloseHandle(hTransaction);
							}
							else
								SET_ERROR2("InternetOpenUrl() error: ");
						}
						else
							SET_ERROR("Input data encode failed");
					}
					else
						SET_ERROR("Failed to grow text buffer");
				}
			}
			else
				SET_ERROR("Input text string too large");
		}
		else
			SET_ERROR("Invalid argument");
	}
	else
		SET_ERROR("Invalid argument");

	#undef SET_ERROR
	#undef SET_ERROR2

	return(rOutput.IsSuccess());
}
