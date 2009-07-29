package org.jbox2d.testbed.mathtests;

import org.jbox2d.common.MathUtils;

public class SinCosTest {
	public static final int NUM_TABLES = 100;
	
	// accuracy
	public static final float MOST_PRECISION = .00001f;
	public static final float LEAST_PRECISION = .01f;
	public static final int ACC_ITERATIONS = 100000;
	
	// speed
	public static final int TRIALS = 20;
	public static final int ITERATIONS = 5000;
		
	// formating stuff
	public static final int COLUMN_PADDING = 2;
	public static final int NUM_DECIMALS = 6;

	private static final SinCosTable[] tables = new SinCosTable[NUM_TABLES];
	/**
	 * @param args
	 */
	public static void main(String[] args) {
		System.out.println("constructing tables");
		for(int i=0; i< NUM_TABLES; i++){
			// well... basic lerp
			float precision = i*1f/NUM_TABLES * (LEAST_PRECISION - MOST_PRECISION) + MOST_PRECISION;
			tables[i] = new SinCosTable(precision);
		}
		
		System.out.println("doing accuracy tests");
		
		double[][] accuracyResults = new double[NUM_TABLES][3];
		
		SinCosTable.LERP_LOOKUP = false;
		// without lerp
		for(int i=0; i< NUM_TABLES; i++){
			accuracyResults[i][0] = accuracyTest(tables[i], ACC_ITERATIONS);				
		}
		
		SinCosTable.LERP_LOOKUP = true;
		// with lerp
		for(int i=0; i< NUM_TABLES; i++){
			accuracyResults[i][1] = accuracyTest(tables[i], ACC_ITERATIONS);				
		}
		
		for(int i=0; i< NUM_TABLES; i++){
			accuracyResults[i][2] = accuracyResults[i][0] - accuracyResults[i][1];
		}
		
		System.out.println("Accuracy results, average displacement");
		String header[] = {
		     "Not lerped", "Lerped", "Difference"
		};
		String side[] = new String[NUM_TABLES+1];
		side[0] = "Table precision";
		for(int i=0; i<tables.length; i++){
			side[i+1] = formatDecimal(tables[i].precision,NUM_DECIMALS);
		}
		printTable(header, side, accuracyResults);

		System.out.println("\nDoing speed tests");
		double[][] speedResults = new double[NUM_TABLES][3];
		
		SinCosTable.LERP_LOOKUP = false;
		// without lerp
		for(int i=0; i< NUM_TABLES; i++){
			speedResults[i][0] = speedTest(tables[i], ITERATIONS, TRIALS);				
		}
		
		SinCosTable.LERP_LOOKUP = true;
		// with lerp
		for(int i=0; i< NUM_TABLES; i++){
			speedResults[i][1] = speedTest(tables[i], ITERATIONS, TRIALS);				
		}
		
		for(int i=0; i< NUM_TABLES; i++){
			speedResults[i][2] = speedResults[i][0] - speedResults[i][1];
		}
		
		System.out.println("Speed results, iterations per second");
  		printTable(header, side, speedResults);
	}

	private static float accuracyTest(SinCosTable table, int iterations){
		float totalDiff = 0f, diff = 0f;
		
		for(int i=0; i<iterations; i++){
			float querry = (float)Math.random()*MathUtils.TWOPI;
			diff = MathUtils.abs((float)Math.sin(querry)-table.sin(querry));
			totalDiff += diff;
		}
		totalDiff /= iterations;
		return totalDiff;
	}
	
	private static void printTable(String header[], String side[], double[][] results){
		
		// first determine the amount of space we need for each column
		int[] colLengths = new int[ results[0].length + 1];
		for(int i=0; i<colLengths.length; i++){
			colLengths[i] = 0;
		}
		for(int j=-1; j < results[0].length; j++){
			if( j == -1){
				int colLength = side[j+1].length() + COLUMN_PADDING;
				if( colLength > colLengths[j+1]){
					colLengths[j+1] = colLength;
				}
			}else{
				int colLength = header[j].length() + COLUMN_PADDING;
				if( colLength > colLengths[j+1]){
					colLengths[j+1] = colLength;
				}
				for( int i=0; i< results.length; i++){
						colLength = (formatDecimal(results[i][j], NUM_DECIMALS)).length() + COLUMN_PADDING;
						if( colLength > colLengths[j+1]){
							colLengths[j+1] = colLength;
						}
				}
			}
			
		}
		
		// header
		
		System.out.print(spaceString(side[0], colLengths[0]));	
		for(int i=1; i< colLengths.length; i++){
			System.out.print(spaceString(header[i-1], colLengths[i]));
		}
		System.out.println();
		
		for(int i=0; i < results.length; i++){
			
			for(int j=-1; j<results[i].length; j++){
				if(j==-1){
					System.out.print(spaceString(side[i+1], colLengths[j+1]));
				}else{
					String toPrint = formatDecimal(results[i][j], NUM_DECIMALS) ;
					System.out.print(spaceString(toPrint, colLengths[j+1]));
				}
			}
			System.out.println();
		}
	}
	
	private static long speedTest(SinCosTable table, final int numIterations, final int numTrials){
		long startTime, endTime;
		long totalTime = 0;
		float i,j;
		
		final float jstep = MathUtils.TWOPI/numIterations;
		
		for(i=0; i<numTrials; i++){
			
			startTime = System.nanoTime();
			for(j=0; j<MathUtils.TWOPI; j+=jstep){
				table.sin(j);
			}
			endTime = System.nanoTime();
			totalTime += endTime - startTime;
		}
		
		return numIterations*numTrials*1000000000l/(totalTime);
	}
	
	public static String spaceString(String str, int space) {
		// if the string is more than the space
		if (str.length() == space) {
			return str;
		}else if(str.length() >= space){
			return str.substring(0, space);
		}
		String s = new String(str);

		for (int i = s.length(); i < space; i++) {
			s = " " + s;
		}
		return s;
	}
	
	public static String formatDecimal(double n, int decimals) {
		String num = n + "";
		// no decimal
		if (num.indexOf(".") == -1) {
			return num;
		}
		
		boolean ePresent = false;
		String e = null;
		
		if(num.indexOf("E") != -1){
			e = num.substring(num.indexOf("E"));
			decimals -= e.length();
			num = num.substring(0, num.indexOf("E"));
			ePresent = true;
		}

		int decLen = num.substring(num.indexOf(".") + 1).length();
		int numLen = num.substring(0, num.indexOf(".")).length();

		// perfect number of decimals
		if (decLen == decimals) {
			return num;
		}

		// if not enough decimals
		if (decLen < decimals) {
			for (int i = 0; i < (decimals - decLen); i++) {
				num = num + " ";
			}
		} else { // more decimals than needed
			num = num.substring(0, numLen + decimals + 1);
		}
		if(ePresent){
			num += e;
		}
		return num;
	}
}
