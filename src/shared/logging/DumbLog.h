#ifndef DUMB_LOG_H
#define DUMB_LOG_H

#include <fnctl.h>

/**
 * Prints messages on stdout or on a given serial port only if DEBUG is defined.
 **/
class DumbLog : public Singleton<Log>, ActiveObject
{

	public: 
		/**
		 * Logs the string on the serial port.
		 * \param msg 	string to log
		 * \return 			0 if succeeded, -1 if there was an error
		 **/
		int log(std::string msg)
		{
			if(msg == NULL || msg.size() == 0)
			{
				return -1;
			}
			
			#ifdef DEBUG
				if(serialFD > 0 && auxSerial)			// Writes on aux serial
				{
					return write( serialFD, msg.c_str(), msg.size() );
				} 
				else 												// Writes on stdout
				{
					return printf( (msg + '\n').c_str() );
				}
			#else
				return 0;
			#endif
		}
		
		/**
		 * Forces the logger to write on a differet serial port.
		 * TODO: check if the serial port is really open.
		 * \arg fd 	serial port file descriptor
		 * \return 	error code or 0 if it succeeded
		 **/
		int setPort( int fd )
		{
			serialFD = fd;
			auxSerial = true;
			return 0;
		}
	
	private:
	    int serialFD = -1;	
		bool auxSerial;
		
		/**
		 * Private constructor ensures that it's a Singleton.
		 **/
		DumbLog()
		{
			auxSerial = false;
		}
}

#ifdef sLog
#error YOU ARE TRYING TO USE MULTIPLE LOGGERS.
#else
#define sLog DumbLog::getInstance()
#endif

#endif /* ifndef DUMB_LOG_H */