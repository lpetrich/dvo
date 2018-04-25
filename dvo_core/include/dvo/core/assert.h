#include <libgen.h>
#include <iostream>

#ifndef ASSERT_H_
#define ASSERT_H_

#define ASSERT_DEBUG

#ifndef ASSERT_DEBUG
#define ASSERT(x)
#else
#define ASSERT(x);\
	if(!(x))\
	{\
		std::cout << "\nERROR!! Assert failed on line " << __LINE__  << " in file " << __FILE__ << "\n";\
	}
#endif /* ASSERT_DEBUG */

#endif /* ASSERT_H_ */
