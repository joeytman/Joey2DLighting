using UnityEngine;
using System.Collections;

public class FastTrigCalculator
{
	//Statically want to have arrays of the values of expensive sine and cosine operations
	private static bool hasInstanced = false;
	private static float granularity = 720f;
	private static float[] SinValues;
	private static float[] CosValues;
	private static float[] TanValues;

	/**
	 * This should only ever be called during loading screen
	 * Does precompute of $granularity num of values for sin and cos
	 */
	public static void memoize(int numVals){
		if(!hasInstanced){
			granularity = (float)numVals;
			instance ();
			for(int i = 0; i < granularity; i++){
				calculateValue (i);
			}
		}		
	}


	private static void instance() {
		SinValues = new float[(int)granularity];
		CosValues = new float[(int)granularity];
		TanValues = new float[(int)granularity];
		hasInstanced = true;
	}

	private static void calculateValue(int indexInArray) {
		float radian = indexToRad (indexInArray);
		SinValues [indexInArray] = Mathf.Sin (radian);
		CosValues [indexInArray] = Mathf.Cos (radian);
		TanValues [indexInArray] = Mathf.Tan (radian);
	}

	public static bool instantiated() {
		return hasInstanced;
	}

		

	/**
	 * These take care of the approximation
	 * Steps:
	 * 1) makes sure we've instantiated the arrays (done during loading)
	 * 2) casts the radian to the closest index in the array
	 * 3) Checks to make sure it has a calculated value, if not, calculates it
	 * 4) Then returns the value of that in the array
	 */
	public static float SinRadApprox(float rad) {
		//1
		if (!hasInstanced) {
			instance ();
		}
		//2
		int indexMap = radToIndex (rad);
		//3
		if (SinValues[indexMap] == 0) {
			calculateValue (indexMap);
		}
		//4
		return SinValues [indexMap];
	}

	public static float CosRadApprox(float rad) {
		//1
		if (!hasInstanced) {
			instance ();
		}
		//2
		int indexMap = radToIndex (rad);
		//3
		if (CosValues[indexMap] == 0) {
			calculateValue (indexMap);
		}
		//4
		return CosValues[indexMap];
	}

	public static float TanRadApprox(float rad) {
		//1
		if (!hasInstanced) {
			instance ();
		}
		//2
		int indexMap = radToIndex (rad);
		//3
		if (TanValues[indexMap] == 0) {
			calculateValue (indexMap);
		}
		//4
		return TanValues[indexMap];
	}
		

	//Essentially alias functions
	public static float CosDegApprox(float degree) {
		return CosRadApprox (degree * Mathf.Deg2Rad);
	}

	public static float SinDegApprox(float degree) {
		return SinRadApprox (degree * Mathf.Deg2Rad);
	}
		
	/**
	 * There are three different psuedoAngle fxns here.
	 * One was written by designer of this
	 * Other two were by me
	 * 
	 */
	public static float getAngleTo(float x, float y, bool pseudo){
		if(pseudo == true){
			return pseudoAngle1(x, y);
		}else{
			return Mathf.Atan2(y, x); //TODO: make sure that this is the right order of arguments
		}
	}

	/**
	 * Option 1 for angle estimation
	 * 
	 * Jump Point: Q4->Q1
	 * Range: [0, 2pi)
	 * Benefits: Sorts things in very defined order, most logical and easy to interpret angle estimator
	 * Drawbacks: Same information conveyed as psuedoAngle2, but more time to calculate
	 * 
	 * Ex vals:
	 *  dx=4, dy=6: real = 0.982, op1 = 0.97
	 *  dx=-4, dy=6: real = 2.159, op1 = 2.17
	 *  dx=-4, dy=-6: real = -2.159 (4.124), op1 = 4.113
	 *  dx=4, dy=-6: real = -0.982 (5.301), op1 = 5.313
	 */
	private static float pseudoAngle1(float dx, float dy){
		float p = psuedoAngle2(dx, dy);
		return (p < 0) ? 6.28f + p: p;
	}

	/**
	 * Option 2 for angle estimation
	 * 
	 * Jump Point: Q2->Q3
	 * Range: [-pi, pi)
	 * Benefits: Simpler code but same accuracy as option 1
	 * Drawbacks: Jump in values at relatively illogical location
	 * 
	 * Ex vals:
	 *  dx=4, dy=6: real = 0.982, op1 = 0.97
	 *  dx=-4, dy=6: real = 2.159, op1 = 2.17
	 *  dx=-4, dy=-6: real = -2.159, op1 = -2.17
	 *  dx=4, dy=-6: real = -0.982, op1 = -0.97
	 */
	private static float psuedoAngle2(float dx, float dy) {
		float p = dx / (Mathf.Abs(dx) + Mathf.Abs(dy));
		return (dy < 0) ? p - 1.57f : 1.57f - p;
	}

	/**
	 * Option 3 for angle estimation
	 * This one was not written by me, and is kept here for posterity's sake/checking discrepency
	 * 
	 * Jump Point: Q3->Q4
	 * Range: (-1, 3)
	 * Benefits: Not really any? It's quick but completely broken
	 * Drawbacks: This literally just doesn't work and has no bearing on the actual angle of the vertices
	 * 
	 * Ex vals:
	 *  dx=4, dy=6: real = 0.982, op3 = 0.6
	 *  dx=-4, dy=6: real = 2.159, op3 = 1.4
	 *  dx=-4, dy=-6: real = -2.159, op3 = 2.6
	 *  dx=4, dy=-6: real = -0.982, op3 = -0.6
	 */

	private static float psuedoAngle3(float dx, float dy) {
		 float ax = Mathf.Abs (dx);
		 float ay = Mathf.Abs (dy);
		 float p = dy / (ax + ay);
		 if (dx < 0){
		 	p = 2 - p;
		 }
		 return p;
	}
		 
	


	//Helpers
	private static int radToIndex(float rad) {
		return (int)(((rad % (2 * Mathf.PI)) / (2 * Mathf.PI)) * granularity);
	}

	private static float indexToRad(int index) {
		return (index % granularity) / granularity * 2 * Mathf.PI;
	}
}

