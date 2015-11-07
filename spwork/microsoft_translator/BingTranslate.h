
// ****************************************************************************
// File: BingTranslate.h
// Desc: Bing/Microsoft Translator support
//
// ****************************************************************************
#pragma once

#include <Wininet.h>
#include <string>
#include <vector>

namespace BingTranslate
{
	struct RESULT
	{
		RESULT() { Clear(); }
		void Clear() {bSuccess = FALSE; Info.clear(); }
		inline void SetSuccess(){ bSuccess = TRUE; }
		inline void SetFail(){ bSuccess = FALSE; }
		inline BOOL IsSuccess(){ return(bSuccess); }

		std::string Info;
		BOOL bSuccess;
	};

	class Authenticate;
	class Translator
	{
	public:
		Translator(__in_z LPCSTR pszID, __in_z LPCSTR pszSecret, __out RESULT &rResult);
		~Translator();

		static BOOL CanConnect(__out RESULT &rOutput);
		BOOL Translate(__in LPCSTR pszInput, int iInputLen, __in_opt LPCSTR pszFromLang, __in LPCSTR pszToLang, __out RESULT &rOutput);

	private:
		BOOL UpdateAuthentication(__out RESULT &rResult);

		HINTERNET    m_hSession;
		Authenticate *m_pcAuth;
		std::string  m_EncodedID, m_EncodedSecret;
		LPSTR		 m_pszBuffer;
		int          m_iBufferSize;
	};
};
