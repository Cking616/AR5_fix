      FR	R	 RR@@ @Ic	+P	 c	c4�      44HH
 HHZ	Z	 ZZ�
.�(/ ��(.�      ..7�      77DD DDL!L! LL$VV VV^!^! ^^~&~$' ~��5�,6 ��-�      --1�      116	�    6	6>�      >>B
B
 BBFF FF
KK KK	P	P	 P`TT TTXX XX\
\
 \\a#a# add	$d"$ ad�*�&+ ���-
2�-*3 � �8�&
2�&.3 ��1,�        ,,A	A	 AACC CCEE EEGG	 GGJJ JML	+@ L	L M	M JMQQ QQSS SSUU UUWW WWY	Y	 YY[
[
 [[] ]  ]]_
"_
" __bb  bb	cc! cc"p	%p#% pp�(�%) ���,�'- ���0�)1 ���	4�+4 ���7�-8 ���9�/: ���	;�	0< ���!
=�!1/ ��+�>�2? ���@�3A ���B�4C ��	�	D�	5E �	�	�	F�	6G �	�	�	H�	7I �	�	�	J�	8K �	�
�
L�
9M �
��
 
N�
 :3 �
�
%�O�;P ���!
N�!<3 ��&�Q�=R ��   S  *:DMZe}�����������������������������������������������������������	�	�	�	�	�
�
�
�
�������M1_statemachine.h Motor_Drive.h Control.h applInterface.h coeappl.h _CiA402_ cia402appl.h ecat_def.h SERVO_CONTROL_WORDS_Bit Switch_On Enable_Voltage Quick_Stop Operation_mode Fault_Reset Halt Reserve Manufacturer_Specific SERVO_CONTROL_WORDS all bit Servo_control SERVO_STATE_WORDS_Bit Ready_Switch Serve_En_ok Fault Voltage_Enabled Switch_Disabled Warning Reserved8 Remote Target_Limit Internal_Limit Target_Value Reserved13 Reserved14_15 SERVO_STATE_WORDS Servo_State LocalAxes CiA402_Init int CiA402_Init(void) CiA402_DeallocateAxis void CiA402_DeallocateAxis(void) CiA402_StateMachine void CiA402_StateMachine(void) CiA402_LocalError void CiA402_LocalError(int) ErrorCode int CiA402_DummyMotionControl void CiA402_DummyMotionControl(int *) pCiA402Axis int * IncFactor CiA402_TransitionAction int CiA402_TransitionAction(int, int *) CiA402_Application void CiA402_Application(int *) Write0xF030 int Write0xF030(int, int, int, int) APPL_AckErrorInd void APPL_AckErrorInd(int) stateTrans APPL_StartMailboxHandler int APPL_StartMailboxHandler(void) APPL_StopMailboxHandler int APPL_StopMailboxHandler(void) APPL_StartInputHandler int APPL_StartInputHandler(int *) APPL_StopInputHandler int APPL_StopInputHandler(void) APPL_StartOutputHandler int APPL_StartOutputHandler(void) APPL_StopOutputHandler int APPL_StopOutputHandler(void) APPL_GenerateMapping int APPL_GenerateMapping(int *, int *) APPL_InputMapping void APPL_InputMapping(int *) pData APPL_OutputMapping void APPL_OutputMapping(int *) APPL_Application void APPL_Application(void)    > $Ak���������������������������	�	�	�	�	�
�
�
�
�
���������������������� c:cia402appl.c@1945@macro@_CiA402_ c:@S@SERVO_CONTROL_WORDS_Bit c:@S@SERVO_CONTROL_WORDS_Bit@FI@Switch_On c:@S@SERVO_CONTROL_WORDS_Bit@FI@Enable_Voltage c:@S@SERVO_CONTROL_WORDS_Bit@FI@Quick_Stop c:@S@SERVO_CONTROL_WORDS_Bit@FI@Operation_mode c:@S@SERVO_CONTROL_WORDS_Bit@FI@Fault_Reset c:@S@SERVO_CONTROL_WORDS_Bit@FI@Halt c:@S@SERVO_CONTROL_WORDS_Bit@FI@Reserve c:@S@SERVO_CONTROL_WORDS_Bit@FI@Manufacturer_Specific c:@U@SERVO_CONTROL_WORDS c:@U@SERVO_CONTROL_WORDS@FI@all c:@U@SERVO_CONTROL_WORDS@FI@bit c:@Servo_control c:@S@SERVO_STATE_WORDS_Bit c:@S@SERVO_STATE_WORDS_Bit@FI@Ready_Switch c:@S@SERVO_STATE_WORDS_Bit@FI@Switch_On c:@S@SERVO_STATE_WORDS_Bit@FI@Serve_En_ok c:@S@SERVO_STATE_WORDS_Bit@FI@Fault c:@S@SERVO_STATE_WORDS_Bit@FI@Voltage_Enabled c:@S@SERVO_STATE_WORDS_Bit@FI@Quick_Stop c:@S@SERVO_STATE_WORDS_Bit@FI@Switch_Disabled c:@S@SERVO_STATE_WORDS_Bit@FI@Warning c:@S@SERVO_STATE_WORDS_Bit@FI@Reserved8 c:@S@SERVO_STATE_WORDS_Bit@FI@Remote c:@S@SERVO_STATE_WORDS_Bit@FI@Target_Limit c:@S@SERVO_STATE_WORDS_Bit@FI@Internal_Limit c:@S@SERVO_STATE_WORDS_Bit@FI@Target_Value c:@S@SERVO_STATE_WORDS_Bit@FI@Reserved13 c:@S@SERVO_STATE_WORDS_Bit@FI@Reserved14_15 c:@U@SERVO_STATE_WORDS c:@U@SERVO_STATE_WORDS@FI@all c:@U@SERVO_STATE_WORDS@FI@bit c:@Servo_State c:@LocalAxes c:@F@CiA402_Init c:@F@CiA402_DeallocateAxis c:@F@CiA402_StateMachine c:@F@CiA402_LocalError c:cia402appl.c@26178@F@CiA402_LocalError@ErrorCode c:@F@CiA402_DummyMotionControl c:cia402appl.c@26825@F@CiA402_DummyMotionControl@pCiA402Axis c:cia402appl.c@26876@F@CiA402_DummyMotionControl@IncFactor c:@F@CiA402_TransitionAction c:@F@CiA402_Application c:cia402appl.c@31058@F@CiA402_Application@pCiA402Axis c:@F@Write0xF030 c:@F@APPL_AckErrorInd c:cia402appl.c@42953@F@APPL_AckErrorInd@stateTrans c:@F@APPL_StartMailboxHandler c:@F@APPL_StopMailboxHandler c:@F@APPL_StartInputHandler c:@F@APPL_StopInputHandler c:@F@APPL_StartOutputHandler c:@F@APPL_StopOutputHandler c:@F@APPL_GenerateMapping c:@F@APPL_InputMapping c:cia402appl.c@56818@F@APPL_InputMapping@pData c:@F@APPL_OutputMapping c:cia402appl.c@61980@F@APPL_OutputMapping@pData c:@F@APPL_Application     @<invalid loc> E:\AR5_Project_EtherCAT\EtherCAT\src\cia402appl.c 