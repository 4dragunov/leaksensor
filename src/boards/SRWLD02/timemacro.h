/*
 * timemacro.h
 *
 *  Created on: Aug 2, 2025
 *      Author: Andrey
 */

#ifndef SRC_BOARDS_SRWLD02_TIMEMACRO_H_
#define SRC_BOARDS_SRWLD02_TIMEMACRO_H_


// helper (Days in February)
#define _UNIX_TIMESTAMP_FDAY(year) \
    (((year) % 400) == 0 ? 29LL : \
        (((year) % 100) == 0 ? 28LL : \
            (((year) % 4) == 0 ? 29LL : \
                28LL)))

// helper (Days in the year)
#define _UNIX_TIMESTAMP_YDAY(year, month, day) \
    ( \
        /* January */    static_cast<uint64_t>(day) \
        /* February */ + ((month) >=  2 ? 31LL : 0LL) \
        /* March */    + ((month) >=  3 ? _UNIX_TIMESTAMP_FDAY(year) : 0LL) \
        /* April */    + ((month) >=  4 ? 31LL : 0LL) \
        /* May */      + ((month) >=  5 ? 30LL : 0LL) \
        /* June */     + ((month) >=  6 ? 31LL : 0LL) \
        /* July */     + ((month) >=  7 ? 30LL : 0LL) \
        /* August */   + ((month) >=  8 ? 31LL : 0LL) \
        /* September */+ ((month) >=  9 ? 31LL : 0LL) \
        /* October */  + ((month) >= 10 ? 30LL : 0LL) \
        /* November */ + ((month) >= 11 ? 31LL : 0LL) \
        /* December */ + ((month) >= 12 ? 30LL : 0LL) \
    )

#define UNIX_TIMESTAMP(year, month, day, hour, minute, second) \
    ( /* time */ static_cast<uint64_t>(second) \
                + static_cast<uint64_t>(minute) * 60LL \
                + static_cast<uint64_t>(hour) * 3600LL \
    + /* year day (month + day) */ (_UNIX_TIMESTAMP_YDAY(year, month, day) - 1) * 86400LL \
    + /* year */ (static_cast<uint64_t>(year) - 1970LL) * 31536000LL \
                + ((static_cast<uint64_t>(year) - 1969LL) / 4LL) * 86400LL \
                - ((static_cast<uint64_t>(year) - 1901LL) / 100LL) * 86400LL \
                + ((static_cast<uint64_t>(year) - 1601LL) / 400LL) * 86400LL )

#endif /* SRC_BOARDS_SRWLD02_TIMEMACRO_H_ */
