- omnetpp im terminal ausführen
- "git ordner"/omnetpp_ws als neuen workspace auswählen
- nur inet installieren
- neues Projekt "inet-ext" mit "Empty project with 'src' ..." anlegen
- beide Project Properties:
	- OMNeT++ -> Makemake:
		- Preview: -I. -I/opt/ros/kinetic/include -L/opt/ros/kinetic/lib -L/usr/lib/x86_64-linux-gnu -lconsole_bridge -lpthread -lboost_atomic -lboost_date_time -lboost_chrono -lboost_thread -lboost_system -lcpp_common -lrostime -lroscpp_serialization -lxmlrpcpp -lboost_regex -llog4cxx -lrosconsole_backend_interface -lrosconsole_log4cxx -lrosconsole -lboost_signals -lboost_filesystem -lroscpp
- inet-ext Project Properties:
	- Project References: Haken bei inet machen
	- OMNeT++ -> Makemake:
		- Compile: Haken bei "Add include paths exported from referenced projects"
- ausführen
!wegen "Project References" muss inet kompiliert sein, bevor man die simulation ausführen kann

nachdem alles kompiliert ist
- git status -s | grep -e "^\?\?" | cut -c 4- >> .gitignore
 
