/**
 * @file      fixed.h
 * @brief     trig functions using fixed point
 * @details   sin and cos functions. Input in decimal fixed point (units radians/1000) or
 * binary fixed point (units 2*pi/16384).
 * Output in either decimal fixed-point (units 1/10000) or binary fixed-point (units 1/65536)
 * @version   TI-RSLK MAX v1.1
 * @author    Daniel Valvano and Jonathan Valvano
 * @copyright Copyright 2019 by Jonathan W. Valvano, valvano@mail.utexas.edu,
 * @warning   AS-IS
 * @note      For more information see  http://users.ece.utexas.edu/~valvano/
 * @date      June 28, 2019
 ******************************************************************************/
#include <stdint.h>

/*!
 * @defgroup Math
 * @brief
 * @{*/
/**
 * decimal fixed-point sin****************
 * @param theta -3142 to 3142,  angle is in units radians/1000
 * @return -10000 to +10000, fixed point resolution 1/10000
 * @note input theta=1571 means 90 degrees (pi/2 radians)
 * @brief decimal fixed-point sin
 */
int16_t fixed_sin(int32_t theta);


/**
 * decimal fixed-point cos****************
 * @param theta -3142 to 3142,  angle is in units radians/1000
 * @return -10000 to +10000, fixed point resolution 1/10000
 * @note input theta=-1571 means 90 degrees (pi/2 radians)
 * @brief decimal fixed-point cos
 */
int16_t fixed_cos(int32_t theta);

/**
 * binary fixed-point sin****************
 * @param theta -8192 to 8191,  angle is in units 2*pi/16384 radians (-pi to +pi)
 * @return -65536 to +65536, fixed point resolution 1/65536
 * @note input theta=-4096 means -90 degrees (pi/2 radians)
 * @brief binary fixed-point sin
 */
int32_t fixed_sin2(int32_t theta);

/**
 * binaryfixed-point cos****************
 * @param theta -8192 to 8191,  angle is in units 2*pi/16384 radians (-pi to +pi)
 * @return  -65536 to +65536, fixed point resolution 1/65536
 * @note input theta=4096 means 90 degrees (pi/2 radians)
 * @brief binary fixed-point cos
 */
int32_t fixed_cos2(int32_t theta);

/**
 * binary fixed-point sin****************<br>
 * e.g., 359.3degrees (2pi radians)  theta=539  sin540(539) = -763   (-0.0116)<br>
 * e.g., 270 degrees (3pi/4 radians) theta=405  sin540(405) = -65536 (-1)<br>
 * e.g., 180 degrees (pi radians)    theta=270  sin540(270) = 0      (0)<br>
 * e.g., 90 degrees (pi/2 radians)   theta=135  sin540(135) = 65536  (+1)<br>
 * e.g., 45 degrees (pi/4 radians)   theta= 67  sin540(67)  = 46071  (sqrt(2)/2)<br>
 * e.g.,  0 degrees (0 radians)      theta= 0   sin540(0)   = 0      (0)<br>
 * @param theta 0 to 539,  angle is in units 2*pi/540 = 0.011635528 radians (0 to 2pi)
 * @return -65536 to +65536, fixed point resolution 1/65536
 * @brief binary fixed-point sin
 */
int32_t sin540(int32_t theta);

/**
 * binaryfixed-point cos****************
 * @param theta 0 to 539,  angle is in units 2*pi/540 = 0.011635528 radians (0 to 2pi)
 * @return  -65536 to +65536, fixed point resolution 1/65536
 * @note input theta=4096 means 90 degrees (pi/2 radians)
 * @brief binary fixed-point cos
 */
int32_t cos540(int32_t theta);
