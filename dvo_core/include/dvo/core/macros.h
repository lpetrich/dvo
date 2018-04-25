/**
 *  This file is part of dvo.
 *
 *  Copyright 2012 Christian Kerl <christian.kerl@in.tum.de> (Technical University of Munich)
 *  For more information see <http://vision.in.tum.de/data/software/dvo>.
 *
 *  dvo is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  dvo is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with dvo.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <libgen.h>
#include <iostream>
#ifndef MACROS_H_
#define MACROS_H_

// #define TRACE_DEBUG

#ifndef TRACE_DEBUG
#define TRACE()
#else
#define TRACE(); \
	std::cout << __LINE__ << " << " << __FUNCTION__ << " (" << __FILE__ << ")\n"; \

#endif /* TRACE_DEBUG */

#endif /* MACROS_H_ */
