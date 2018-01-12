;
;#ifndef _RIFFALOG_
;#define _RIFFALOG_
;
;//
;//  Status values are 32 bit values layed out as follows:
;//
;//   3 3 2 2 2 2 2 2 2 2 2 2 1 1 1 1 1 1 1 1 1 1
;//   1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0
;//  +---+-+-------------------------+-------------------------------+
;//  |Sev|C|       Facility          |               Code            |
;//  +---+-+-------------------------+-------------------------------+
;//
;//  where
;//
;//      Sev - is the severity code
;//
;//          00 - Success
;//          01 - Informational
;//          10 - Warning
;//          11 - Error
;//
;//      C - is the Customer code flag
;//
;//      Facility - is the facility code
;//
;//      Code - is the facility's status code
;//
;
MessageIdTypedef=NTSTATUS

SeverityNames=(Success=0x0:STATUS_SEVERITY_SUCCESS
               Informational=0x1:STATUS_SEVERITY_INFORMATIONAL
               Warning=0x2:STATUS_SEVERITY_WARNING
               Error=0x3:STATUS_SEVERITY_ERROR
              )

FacilityNames=(System=0x0
               RIFFA =0x1:FACILITY_RIFFA_ERROR_CODE
              )


MessageId=0x0001 Facility=RIFFA Severity=Informational SymbolicName=RIFFA_INITIALIZATION_MESSAGE
Language=English
RIFFA device initialized.
.

MessageId=0x0002 Facility=RIFFA Severity=Error SymbolicName=RIFFA_ACCESS_CLOSED_DMA
Language=English
An attempt was made by channel %2 to write data to a closed DMA. Most likely the FPGA 
wrote data to a TX that has timed out.
.

;#endif /* _RIFFALOG_ */

