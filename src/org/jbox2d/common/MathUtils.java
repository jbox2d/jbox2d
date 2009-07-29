/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/
 * Box2D homepage: http://www.box2d.org
 * 
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package org.jbox2d.common;

/**
 * A few math methods that don't fit very well anywhere else. djm: added ToOut
 * method
 */
public class MathUtils {
	public static final float PI = (float) Math.PI;
	public static final float TWOPI = (float) (Math.PI * 2);
		
	
	public static final float[] sinLUT = new float[Settings.SINCOS_LUT_LENGTH];
	public static final float[] cosLUT = new float[Settings.SINCOS_LUT_LENGTH];
	
	static {
		for(int i=0; i<Settings.SINCOS_LUT_LENGTH; i++){
			sinLUT[i] = (float) Math.sin( i * Settings.SINCOS_LUT_PRECISION);
			cosLUT[i] = (float) Math.cos( i * Settings.SINCOS_LUT_PRECISION);
		}
	}
	
	public static final float sin(float x){
		if(Settings.SINCOS_LUT_ENABLED){
			x %= TWOPI;
			
			while(x < 0){
				x += TWOPI;
			}
			
			if(Settings.SINCOS_LUT_LERP){
				
				x /= Settings.SINCOS_LUT_PRECISION;
								
				final int index = (int)x;
				
				if(index != 0){
					x %= index;
				}
				
				// the next index is 0
				if(index == Settings.SINCOS_LUT_LENGTH-1){
					return ( (1-x)*sinLUT[index] + x * sinLUT[0]);
				}else{
					return ( (1-x)*sinLUT[index] + x * sinLUT[index + 1]);
				}
				
			}else{
				return sinLUT[ MathUtils.round(x / Settings.SINCOS_LUT_PRECISION) % Settings.SINCOS_LUT_LENGTH];
			}
			
		}else{
			return (float) Math.sin(x);
		}
	}
	
	public static final float cos(float x){
		if(Settings.SINCOS_LUT_ENABLED){
			x %= TWOPI;
			
			while(x < 0){
				x += TWOPI;
			}
			
			if(Settings.SINCOS_LUT_LERP){
				
				x /= Settings.SINCOS_LUT_PRECISION;
								
				final int index = (int)x;
				
				if(index != 0){
					x %= index;
				}
				
				// the next index is 0
				if(index == Settings.SINCOS_LUT_LENGTH-1){
					return ( (1-x)*cosLUT[index] + x * cosLUT[0]);
				}else{
					return ( (1-x)*cosLUT[index] + x * cosLUT[index + 1]);
				}
				
			}else{
				return cosLUT[ MathUtils.round(x / Settings.SINCOS_LUT_PRECISION) % Settings.SINCOS_LUT_LENGTH];
			}
			
		}else{
			return (float) Math.cos(x);
		}
	}

	public static final float abs(final float x) {
		if (Settings.FAST_MATH) {
			return x > 0 ? x : -x;
		}
		else {
			return Math.abs(x);
		}
	}
	
	public static final int floor(final float x) {
		if (Settings.FAST_MATH) {
			return x > 0 ? (int) x : (int) x - 1;
		}
		else {
			return (int) Math.floor(x);
		}
	}
	
	public static final int ceil(final float x){
		if (Settings.FAST_MATH){
			return floor(x+.5f);
		}else{
			return (int) Math.ceil(x);
		}
	}
	
	public static final int round(final float x){
		if(Settings.FAST_MATH){
			return floor(x + .5f);
		}else{
			return Math.round(x);
		}
	}

	// Max/min rewritten here because for some reason MathUtils.max/min
	// can run absurdly slow for such simple functions...
	// TODO: profile, see if this just seems to be the case or is actually
	// causing issues...
	public final static float max(final float a, final float b) {
		return a > b ? a : b;
	}

	public final static int max(final int a, final int b) {
		return a > b ? a : b;
	}

	public final static float min(final float a, final float b) {
		return a < b ? a : b;
	}

	public final static float map(final float val, final float fromMin, final float fromMax,
			final float toMin, final float toMax) {
		final float mult = (val - fromMin) / (fromMax - fromMin);
		final float res = toMin + mult * (toMax - toMin);
		return res;
	}

	/** Returns the closest value to 'a' that is in between 'low' and 'high' */
	public final static float clamp(final float a, final float low, final float high) {
		return max(low, min(a, high));
	}

	/* djm optimized */
	public final static Vec2 clamp(final Vec2 a, final Vec2 low, final Vec2 high) {
		final Vec2 min = new Vec2();
		Vec2.minToOut(a, high, min);
		Vec2.maxToOut(low, min, min);
		return min;
	}

	/* djm created */
	public final static void clampToOut(final Vec2 a, final Vec2 low, final Vec2 high,
			final Vec2 dest) {
		Vec2.minToOut(a, high, dest);
		Vec2.maxToOut(low, dest, dest);
	}

	/**
	 * Next Largest Power of 2: Given a binary integer value x, the next largest
	 * power of 2 can be computed by a SWAR algorithm that recursively "folds"
	 * the upper bits into the lower bits. This process yields a bit vector with
	 * the same most significant 1 as x, but all 1's below it. Adding 1 to that
	 * value yields the next largest power of 2.
	 */
	public final static int nextPowerOfTwo(int x) {
		x |= x >> 1;
		x |= x >> 2;
		x |= x >> 4;
		x |= x >> 8;
		x |= x >> 16;
		return x + 1;
	}

	public final static boolean isPowerOfTwo(final int x) {
		return x > 0 && (x & x - 1) == 0;
	}

	// UNTESTED
	public static final float atan2(final float y, final float x) {
		if (Settings.FAST_MATH) {
			// float coeff_1 = PI/4;
			// float coeff_2 = 3*coeff_1;
			final float abs_y = abs(y) + .0000000001f; // kludge to prevent 0/0
			// condition
			float angle, r;
			if (x >= 0) {
				r = (x - abs_y) / (x + abs_y);
				// angle = coeff_1 - coeff_1 * r;
				angle = 0.1963f * r * r * r - 0.9817f * r + Settings.pi / 4;
			}
			else {
				r = (x + abs_y) / (abs_y - x);
				// angle = coeff_2 - coeff_1 * r;
				angle = 0.1963f * r * r * r - 0.9817f * r + 3 * Settings.pi / 4;
			}
			if (y < 0) {
				return -angle; // negate if in quad III or IV
			}
			else {
				return angle;
			}
		}
		else {
			return (float) Math.atan2(y, x);
		}
	}
	
	

	/**
	 * Computes a fast approximation to <code>Math.pow(a, b)</code>.
	 * Adapted from <url>http://www.dctsystems.co.uk/Software/power.html</url>.
	 * 
	 * @param a
	 *            a positive number
	 * @param b
	 *            a number
	 * @return a^b
	 */
	// UNTESTED
	public static final float pow(final float a, float b) {
		// adapted from: http://www.dctsystems.co.uk/Software/power.html
		if (Settings.FAST_MATH) {
			float x = Float.floatToRawIntBits(a);
			x *= 1.0f / (1 << 23);
			x = x - 127;
			float y = x - MathUtils.floor(x);
			b *= x + (y - y * y) * 0.346607f;
			y = b - MathUtils.floor(b);
			y = (y - y * y) * 0.33971f;
			return Float.intBitsToFloat((int) ((b + 127 - y) * (1 << 23)));
		}
		else {
			return (float) Math.pow(a, b);
		}
	}

	public static final float sqrt(float x) {
		if (Settings.FAST_MATH) {
			x = invSqrt(x);

			if (x != 0.0f) {
				return 1.0f / x;
			}
			else {
				return 0;
			}
		}
		else {
			return (float) Math.sqrt(x);
		}
	}

	public final static float invSqrt(float x) {
		final float xhalf = 0.5f * x;
		int i = Float.floatToRawIntBits(x);
		i = 0x5f3759df - (i >> 1);
		x = Float.intBitsToFloat(i);
		x *= 1.5f - xhalf * x * x;
		// REPEAT FOR ACCURACY (make sure at least 2 are here, too inaccurate
		// otherwise)
		x *= 1.5f - xhalf * x * x;
		x *= 1.5f - xhalf * x * x;
		x *= 1.5f - xhalf * x * x;
		return x;
	}
}
// SINCOS accuracy and speed chart
// 
//constructing tables
//doing accuracy tests
//Accuracy results, average displacement
//  Table precision  Not lerped    Lerped  Difference
//         9.999E-6    1.595E-6  6.325E-8    1.532E-6
//         1.098E-4    1.747E-5  5.520E-8    1.742E-5
//         2.097E-4    3.351E-5  6.019E-8    3.345E-5
//         3.097E-4    4.936E-5  6.162E-8    4.930E-5
//         4.095E-4    6.492E-5  6.184E-8    6.486E-5
//         5.095E-4    8.112E-5  5.909E-8    8.106E-5
//         6.094E-4    9.684E-5  7.413E-8    9.677E-5
//         7.093E-4    1.130E-4  7.028E-8    1.129E-4
//         8.091E-4    1.284E-4  7.661E-8    1.283E-4
//         9.091E-4    1.444E-4  8.847E-8    1.443E-4
//         0.001008    1.601E-4  8.848E-8    1.600E-4
//         0.001108    1.767E-4  1.008E-7    1.766E-4
//         0.001208    1.917E-4  1.214E-7    1.915E-4
//         0.001308    2.080E-4  1.189E-7    2.078E-4
//         0.001408    2.245E-4  1.667E-7    2.244E-4
//         0.001508    2.403E-4  1.461E-7    2.401E-4
//         0.001608    2.561E-4  1.909E-7    2.559E-4
//         0.001708    2.712E-4  1.702E-7    2.710E-4
//         0.001808    2.882E-4  2.191E-7    2.879E-4
//         0.001908    3.029E-4  2.269E-7    3.027E-4
//         0.002008    3.199E-4  2.297E-7    3.196E-4
//         0.002107    3.373E-4  2.923E-7    3.370E-4
//         0.002207    3.522E-4  3.161E-7    3.519E-4
//         0.002307    3.690E-4  3.514E-7    3.686E-4
//         0.002407    3.832E-4  3.943E-7    3.828E-4
//         0.002507    4.000E-4  3.962E-7    3.996E-4
//         0.002607    4.135E-4  4.307E-7    4.131E-4
//         0.002707    4.297E-4  4.820E-7    4.293E-4
//         0.002807    4.450E-4  4.460E-7    4.446E-4
//         0.002907    4.611E-4  5.099E-7    4.606E-4
//         0.003007    4.774E-4  5.813E-7    4.769E-4
//         0.003106    4.956E-4  6.007E-7    4.950E-4
//         0.003206    5.108E-4  6.093E-7    5.102E-4
//         0.003306    5.269E-4  5.961E-7    5.263E-4
//         0.003406    5.429E-4  7.054E-7    5.422E-4
//         0.003506    5.602E-4  7.604E-7    5.595E-4
//         0.003606    5.749E-4  7.512E-7    5.742E-4
//         0.003706    5.891E-4  7.871E-7    5.883E-4
//         0.003806    6.030E-4  9.711E-7    6.020E-4
//         0.003906    6.207E-4  9.480E-7    6.198E-4
//         0.004006    6.406E-4  1.001E-6    6.396E-4
//         0.004105    6.519E-4  9.631E-7    6.510E-4
//         0.004205    6.729E-4  9.872E-7    6.719E-4
//         0.004305    6.840E-4  1.063E-6    6.829E-4
//         0.004405    7.024E-4  1.060E-6    7.013E-4
//         0.004505    7.152E-4  1.229E-6    7.140E-4
//         0.004605    7.321E-4  1.227E-6    7.309E-4
//         0.004705    7.500E-4  1.312E-6    7.487E-4
//         0.004805    7.660E-4  1.502E-6    7.645E-4
//         0.004905    7.819E-4  1.361E-6    7.806E-4
//         0.005005    7.953E-4  1.549E-6    7.938E-4
//         0.005104    8.118E-4  1.640E-6    8.102E-4
//         0.005204    8.334E-4  1.508E-6    8.319E-4
//         0.005304    8.435E-4  1.833E-6    8.417E-4
//         0.005404    8.658E-4  1.982E-6    8.638E-4
//         0.005504    8.761E-4  1.922E-6    8.741E-4
//         0.005604    8.894E-4  1.722E-6    8.877E-4
//         0.005704    9.102E-4  2.068E-6    9.082E-4
//         0.005804    9.234E-4  2.070E-6    9.214E-4
//         0.005904    9.390E-4  1.925E-6    9.371E-4
//         0.006004    9.583E-4  2.247E-6    9.561E-4
//         0.006103    9.718E-4  2.258E-6    9.695E-4
//         0.006203    9.923E-4  2.454E-6    9.898E-4
//         0.006303    0.001001  2.570E-6    9.989E-4
//         0.006403    0.001020  2.240E-6    0.001018
//         0.006503    0.001031  2.290E-6    0.001029
//         0.006603    0.001051  2.689E-6    0.001048
//         0.006703    0.001067  2.630E-6    0.001064
//         0.006803    0.001083  2.925E-6    0.001080
//         0.006903    0.001097  2.639E-6    0.001094
//         0.007003    0.001109  2.761E-6    0.001106
//         0.007102    0.001125  3.134E-6    0.001122
//         0.007202    0.001140  3.105E-6    0.001137
//         0.007302    0.001165  3.170E-6    0.001161
//         0.007402    0.001175  3.373E-6    0.001172
//         0.007502    0.001192  3.417E-6    0.001189
//         0.007602    0.001214  3.708E-6    0.001210
//         0.007702    0.001222  3.737E-6    0.001219
//         0.007802    0.001245  3.577E-6    0.001241
//         0.007902    0.001253  3.395E-6    0.001250
//         0.008001    0.001278  3.519E-6    0.001274
//         0.008101    0.001284  4.303E-6    0.001279
//         0.008201    0.001308  3.609E-6    0.001305
//         0.008301    0.001319  4.201E-6    0.001315
//         0.008401    0.001339  4.281E-6    0.001334
//         0.008501    0.001351  3.871E-6    0.001347
//         0.008601    0.001367  4.697E-6    0.001362
//         0.008701    0.001380  4.065E-6    0.001376
//         0.008801    0.001398  4.656E-6    0.001393
//         0.008901    0.001413  4.822E-6    0.001408
//         0.009000    0.001434  4.339E-6    0.001429
//         0.009100    0.001452  4.935E-6    0.001447
//         0.009200    0.001465  5.108E-6    0.001460
//         0.009300    0.001474  5.485E-6    0.001468
//         0.009400    0.001491  5.416E-6    0.001486
//         0.009500    0.001512  5.305E-6    0.001507
//         0.009600    0.001532  5.770E-6    0.001526
//         0.009700    0.001541  5.998E-6    0.001535
//         0.009800    0.001563  5.155E-6    0.001558
//         0.009900    0.001574  6.540E-6    0.001568
//
//Doing speed tests
//Speed results, iterations per second
//  Table precision  Not lerped          Lerped       Difference
//         9.999E-6    1.2227E7        1.1176E7   1050998.0     
//         1.098E-4    1.6923E7        1.3413E7   3509521.0     
//         2.097E-4    1.7262E7        1.4480E7   2782051.0     
//         3.097E-4    1.7534E7        1.4359E7   3175067.0     
//         4.095E-4    1.7503E7        1.4600E7   2903267.0     
//         5.095E-4    1.7661E7        1.4705E7   2955721.0     
//         6.094E-4    1.7079E7        1.4602E7   2476616.0     
//         7.093E-4    1.7461E7        1.4530E7   2930489.0     
//         8.091E-4    1.1999E7        1.4427E7  -2428892.0     
//         9.091E-4    1.6739E7        1.4558E7   2181044.0     
//         0.001008    1.6028E7        1.4740E7   1287643.0     
//         0.001108    1.6305E7        1.4634E7   1670373.0     
//         0.001208    1.6778E7        1.4662E7   2115767.0     
//         0.001308    1.6641E7        1.4575E7   2066570.0     
//         0.001408    1.7146E7        1.4594E7   2552497.0     
//         0.001508    1.7528E7        1.3894E7   3633805.0     
//         0.001608    1.7664E7        1.4686E7   2978279.0     
//         0.001708    1.7683E7        1.4645E7   3037888.0     
//         0.001808    1.7917E7        1.4543E7   3374596.0     
//         0.001908    1.7652E7        1.4731E7   2920371.0     
//         0.002008    1.7892E7        1.4677E7   3214467.0     
//         0.002107    1.7593E7        1.4632E7   2960526.0     
//         0.002207    1.7914E7        1.4551E7   3362921.0     
//         0.002307    1.7664E7        1.4536E7   3127727.0     
//         0.002407    1.7120E7        1.4579E7   2540972.0     
//         0.002507    1.6614E7        1.4688E7   1925454.0     
//         0.002607    1.7525E7        1.4564E7   2960891.0     
//         0.002707    1.6863E7        1.4699E7   2164009.0     
//         0.002807    1.7559E7        1.4602E7   2956459.0     
//         0.002907    1.6647E7        1.4628E7   2018807.0     
//         0.003007    1.2448E7        1.3424E7   -975971.0     
//         0.003106    1.7818E7        1.4667E7   3151902.0     
//         0.003206    1.7692E7        1.4573E7   3119842.0     
//         0.003306    1.7777E7        1.4628E7   3149340.0     
//         0.003406    1.7758E7        1.4705E7   3052953.0     
//         0.003506    1.7838E7        1.4590E7   3248010.0     
//         0.003606    1.7761E7        1.4583E7   3178352.0     
//         0.003706    1.7784E7        1.4526E7   3257663.0     
//         0.003806    1.7667E7        1.4664E7   3002938.0     
//         0.003906    1.7714E7        1.4628E7   3086354.0     
//         0.004006    1.7027E7        1.4607E7   2420004.0     
//         0.004105    1.7758E7        1.4697E7   3061599.0     
//         0.004205    1.7825E7        1.3757E7   4068261.0     
//         0.004305    1.6880E7        1.3869E7   3010861.0     
//         0.004405    1.7934E7        1.4390E7   3543443.0     
//         0.004505    1.7777E7        1.4255E7   3522610.0     
//         0.004605    1.7577E7        1.2629E7   4948330.0     
//         0.004705    1.6960E7        1.4013E7   2947199.0     
//         0.004805    1.7934E7  9944311.0        7989691.0     
//         0.004905    1.6772E7        1.2518E7   4254117.0     
//         0.005005    1.7473E7        1.3612E7   3860503.0     
//         0.005104    1.7108E7        1.0610E7   6498560.0     
//         0.005204    1.4338E7        1.3073E7   1265369.0     
//         0.005304    1.6528E7        1.2430E7   4098845.0     
//         0.005404    1.3003E7  6911327.0        6092574.0     
//         0.005504    1.7473E7  6090133.0              1.1383E7
//         0.005604    1.7593E7  9531071.0        8062173.0     
//         0.005704    1.7164E7        1.1860E7   5303446.0     
//         0.005804    1.7730E7        1.2405E7   5325088.0     
//         0.005904    1.7787E7        1.0132E7   7654526.0     
//         0.006004    1.7844E7        1.2396E7   5448215.0     
//         0.006103    1.7596E7        1.2379E7   5217038.0     
//         0.006203    1.7857E7        1.1522E7   6335078.0     
//         0.006303    1.7895E7        1.3958E7   3936808.0     
//         0.006403    1.7692E7        1.4566E7   3126210.0     
//         0.006503    1.7445E7        1.4029E7   3416737.0     
//         0.006603    1.7873E7  9535615.0        8337485.0     
//         0.006703    1.7602E7        1.4423E7   3178764.0     
//         0.006803    1.7796E7        1.4230E7   3565937.0     
//         0.006903    1.7780E7        1.4234E7   3546063.0     
//         0.007003    1.7736E7        1.3760E7   3975950.0     
//         0.007102    1.7793E7        1.4174E7   3619250.0     
//         0.007202    1.7882E7        1.3010E7   4872021.0     
//         0.007302    1.7863E7        1.4539E7   3324412.0     
//         0.007402    1.7771E7        1.4649E7   3121591.0     
//         0.007502    1.7537E7        1.4551E7   2985902.0     
//         0.007602    1.7866E7        1.4736E7   3130493.0     
//         0.007702    1.7667E7        1.4675E7   2992177.0     
//         0.007802    1.7784E7        1.4630E7   3153524.0     
//         0.007902    1.6450E7        1.4721E7   1729038.0     
//         0.008001    1.7702E7        1.4432E7   3270152.0     
//         0.008101    1.7132E7        1.4677E7   2454267.0     
//         0.008201    1.7758E7        1.4543E7   3215496.0     
//         0.008301    1.7768E7        1.4675E7   3092634.0     
//         0.008401    1.7301E7        1.4423E7   2877268.0     
//         0.008501    1.7334E7        1.4471E7   2862246.0     
//         0.008601    1.7905E7        1.4695E7   3210025.0     
//         0.008701    1.7550E7        1.4673E7   2876503.0     
//         0.008801    1.7943E7        1.4654E7   3289495.0     
//         0.008901    1.7972E7        1.4675E7   3297014.0     
//         0.009000    1.7761E7        1.4598E7   3163449.0     
//         0.009100    1.7905E7        1.4699E7   3205705.0     
//         0.009200    1.7962E7        1.4607E7   3355927.0     
//         0.009300    1.7711E7        1.4684E7   3027367.0     
//         0.009400    1.8076E7        1.4218E7   3857961.0     
//         0.009500    1.8047E7        1.4347E7   3700081.0     
//         0.009600    1.8086E7        1.4505E7   3581087.0     
//         0.009700    1.7914E7        1.4326E7   3588078.0     
//         0.009800    1.7895E7        1.3999E7   3895770.0     
//         0.009900    1.7879E7        1.2196E7   5682883.0     
