// kommando interpreter für scpi kommandolisten

#include <stdlib.h>
#include <qstring.h>
#include <qstringlist.h>
#include "cmdinterpret.h"
#include "scpi.h"
#include "zeraglobal.h"
#include "wmscpi.h"


char* cCmdInterpreter::CmdExecute(char* s) {
    SetAnswer(ACKString);
    dedicatedList.clear();
    if (*s) { // leeres kommando ist nichts falsches -> also richtig
	char* CmdString=s; // der input string
	cNode* actNode=m_pRootCmd; // startknoten setzen
	cNode* prevNode;
	do {
	    prevNode=actNode;
	} while ( (actNode = actNode->TestNode(this,&CmdString)) );
	switch ( prevNode->m_nNodeStat ) {
	  case (isKnown | isCommand) : SetAnswer( m_pcbIFace->SCPICmd(prevNode->m_nCmd,CmdString));break;
	  case (isKnown | isQuery) : SetAnswer( m_pcbIFace->SCPIQuery(prevNode->m_nQuery,CmdString));break;
	  case (isKnown | isQuerywPar) : SetAnswer( m_pcbIFace->SCPIQuery(prevNode->m_nQuery,CmdString));break;    
	  default: SetAnswer(NACKString);
	  };
    }
    return (Answer);
}


void cCmdInterpreter::SetAnswer(const char* s)
{
    if (Answer) free(Answer);
    Answer=(char*) malloc(strlen(s)+1);
    strcpy(Answer,s);
}
