#pragma once

#include <stdint.h>
#include <stddef.h>

#include "ul/list/list.hpp"
#include "FreeRTOS.h"
#include "semphr.h"

// Macro for easy and readable use of the system log
#define log() ul::syslog::get_instance()

// Max string size including timestamp, tag and end of line (\r\n)
#define MAX_STRING_SIZE 200

namespace ul
{
class syslog
{
	public:
		static syslog &get_instance()
		{
			static syslog instance;
			return instance;
		}
		
		typedef void (*cb_t)(uint8_t *buff, size_t size, void *ctx);
		void add_output(cb_t cb, void *ctx = NULL);
		void del_output(cb_t cb, void *ctx = NULL);
		
		// Do not call these APIs from ISR
		void dbg(const char *format, ...);
		void inf(const char *format, ...);
		void wrn(const char *format, ...);
		void err(const char *format, ...);
		
		syslog(syslog const &) = delete;
		void operator =(syslog const &) = delete;
		
	private:
		syslog();
		//syslog(syslog const &);
		//void operator =(syslog const &);
		
		static syslog *instance;
		SemaphoreHandle_t api_lock;
		
		struct cb_ctx_t
		{
			cb_t cb;
			void *ctx;
			
			// Overload "==" operator to use .find() methon of the list
			bool operator==(const cb_ctx_t &a) const
			{
				return (cb == a.cb && ctx == a.ctx);
			}
		};
		list<cb_ctx_t> cb_ctx_list;
		
		enum tag_t
		{
			DBG,
			INF,
			WRN,
			ERR
		};
		char message[MAX_STRING_SIZE];
		void print(tag_t tag, const char *format, va_list va);
};
}
