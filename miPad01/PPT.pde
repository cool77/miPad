/*
this is a class handels the trasformation of points from skewed plane to a normal one
it recives on constuction 8 points, actually 2 sets of 4 points. 4 on skewed plan nad 4 on normal
*/



public class Point2D { // It's just a data structure
		public Point2D(double x, double y){
			m_x = x;
			m_y = y;
		}
		public Point2D(){
			m_x = 0.;
			m_y = 0.;
		}
		public void set(double x, double y){
			m_x = x;
			m_y = y;
		}
		
		public double m_x;
		public double m_y;
	}

class PlanarProjectiveTransform {
	
	// PUBLIC INNER CLASSES
	
	/**
	 * The points submitted to the transformation are embodied in Point2D objects
	 */
	
	
	
	
	// PUBLIC METHODS
	
	public PlanarProjectiveTransform(int n_pts, double real_pts[][], double proj_pts[][]){
		
		// All the body of the constructor implements the estimation of the 8 parameters of the Transformation
		// Eventually these parameters are stored in m_H 3 by 3 array (matrix). The last 9-th element 
		// m_H[2][2] is filled by 1.
		// The inverse transformation parameters are calculted by inverse of the m_H matrix and stored in 
		// m_invH
		
		
		
		// Construct the equation matrix
		
		double A[][] = new double [2*n_pts][8]; 
		for (int pt=0, row=0; pt<n_pts; pt++, row++) {
			
			A[row][0]=real_pts[pt][0];
			A[row][1]=real_pts[pt][1];
			A[row][2]=1.;
			A[row][3]=A[row][4]=A[row][5]=0.;
			A[row][6] = -real_pts[pt][0]*proj_pts[pt][0];
			A[row][7] = -real_pts[pt][1]*proj_pts[pt][0];
			
			row++;
			
			A[row][0]=A[row][1]=A[row][2]=0.;
			A[row][3]=real_pts[pt][0];
			A[row][4]=real_pts[pt][1];
			A[row][5]=1.;			
			A[row][6] = -real_pts[pt][0]*proj_pts[pt][1];
			A[row][7] = -real_pts[pt][1]*proj_pts[pt][1];			
			
		}
		// ATA:
		double ATA[][] = new double[8][8];
		for (int i=0;i<8;i++){
			for (int j=0; j<8; j++){
				ATA[i][j]=0.;
				for (int k=0; k<8; k++) {
					ATA[i][j] += A[k][i]*A[k][j];
				}
			}
		}
		
		// Construct the right-side vector
		double b[] = new double [2*n_pts];
		for (int pt=0, i=0; pt<n_pts; pt++){
			b[i++]=proj_pts[pt][0];
			b[i++]=proj_pts[pt][1];
		}
		// ATb:
		double ATb[] = new double [8];
		for (int i=0; i<8; i++){
			ATb[i]=0.;
			for (int k=0; k<2*n_pts; k++){
				ATb[i]+=A[k][i]*b[k];				
			}
		}
		
		// Solve the equations ATA*h = ATb
		double buf[] = new double [8];
		int indx[] = new int [8];
		boolean success = LU_Decomposition(ATA,8,indx,buf);
	    if (!success) {
	    	println( "BadCalibrationConditionsException:At LU decomposition attempt");	    	
	    }
	    LU_Solve(ATA,8,indx,ATb);
	    
	    // Reshape the solution to form the projective transform matrix		
		m_H = new double[3][3];
		m_H[0][0] = ATb[0]; m_H[0][1] = ATb[1]; m_H[0][2] = ATb[2];
		m_H[1][0] = ATb[3]; m_H[1][1] = ATb[4]; m_H[1][2] = ATb[5];
		m_H[2][0] = ATb[6]; m_H[2][1] = ATb[7]; m_H[2][2] = 1.;
		
		// Create the inverse projective transform matrix		
		m_invH = new double[3][3];
		double LU[][] = new double[3][3];
		for (int i=0; i<3; i++) {
			for (int j=0; j<3; j++){
				LU[i][j] = m_H[i][j];
			}
		}
		success = LU_Decomposition(LU,3,indx,buf);
	    if (!success) {
	    	println("BadCalibrationConditionsException: At transformation matrix inversion attempt");	    	
	    }
	    // 1st column: 
	    buf[0]=1.; buf[1]=0.; buf[2]=0.;
	    LU_Solve(LU,3,indx,buf);
	    for (int i=0; i<3; i++){
	    	m_invH[i][0]=buf[i];
	    }
	    // 2nd column: 
	    buf[0]=0.; buf[1]=1.; buf[2]=0.;
	    LU_Solve(LU,3,indx,buf);
	    for (int i=0; i<3; i++){
	    	m_invH[i][1]=buf[i];
	    }
	    // 3rd column: 
	    buf[0]=0.; buf[1]=0.; buf[2]=1.;
	    LU_Solve(LU,3,indx,buf);
	    for (int i=0; i<3; i++){
	    	m_invH[i][2]=buf[i];
	    }
		
	}
	
	/**
	 * This method projects the given real point to the camera plane.
	 * @param real - The real point to be projected.
	 * @return The projection point at the camera plane.
	 */
	public Point2D RealToProj(Point2D real) {		
			
		Point2D proj = apply_proj_transform(m_H,real);		
		return proj;
	}
	
	/**
	 * This method reconstructs the original real point corresponding to the given point at the camera plane
	 * @param proj - The point at the camera plane
	 * @return The reconstructed original real point 
	 */
	public Point2D ProjToReal(Point2D proj) {		
		
		Point2D real = apply_proj_transform(m_invH,proj);		
		return real;
	}
	
	
	
	
	// HELPER METHODS
	
	
	private Point2D apply_proj_transform(double mat[][], Point2D inp) {		
		
		double ext_inp[] = new double[3];
		ext_inp[0] = inp.m_x;
		ext_inp[1] = inp.m_y;
		ext_inp[2] = 1.;		
		double den = scalar_product(mat, ext_inp, 3, 2);
		double x = scalar_product(mat, ext_inp, 3, 0)/den;
		double y = scalar_product(mat, ext_inp, 3, 1)/den;
		
		Point2D out = new Point2D(x,y);
		return out;
	}
	
	
	/*
	 * Calculate scalar product of matrix's row by a vector
	 */
	private double scalar_product(double mat[][], double vec[], int dim, int row) {
		double sp = 0;
		for (int i=0; i<dim; i++) {
			sp += mat[row][i]*vec[i];
		}
		return sp;
	}
	
	private double fabs(double val){
		return (val>=0. ? val : -val);
	}
	

	
	// FIELDS
	
	private double m_H[][]; // Forward Projective Transformation matrix
	private double m_invH[][]; // Inverse Projective Transformation matrix

	
	// Following two functions were fetched from a pre-existing C-code.
	// They implement a method for solving a system of linear equations called LU Decomposition.
	// These functions have a generic value and in the future should be organized in a separate
	// class. For the moment I keep them here.

	/***********************************************************************
	LU decomposition of the NxN matrix A.
	  Input:
	A[N][N] - the matrix
	WrkBuf - work space allocated for N elements.
	  Output:
	A - replaced by the matrix containing L/U triangles.
	Indx - vector containing the rows permuatation indeces
	  Return:
	true - done
	false - the matrix is singular
	*********************************************************************/
	private boolean LU_Decomposition(
	        double  A[][],
	        int    N,
	        int    Indx[],
	        double  WrkBuf[]
	        )
	{
	double fmax, sum, dum;
	int i, j, k, imax=0;

	for (i=0; i<N; i++) {
	    fmax = 0.f;
	    for (j=0; j<N; j++) {
	        if (fabs(A[i][j]) > fmax)
	            fmax = (double)fabs(A[i][j]);
	    }
	    if (0 == fmax) return false;
	    WrkBuf[i] = 1.f/fmax;
	}

	// Loop over all the columns
	for (j=0; j<N; j++) {

	    for (i=0; i<j; i++) {
	        sum = A[i][j];
	        for (k=0; k<i; k++)
	            sum -= A[i][k]*A[k][j];
	        A[i][j] = sum;
	    }

	    fmax=0.f;
	    for (i=j; i<N; i++) {
	        sum = A[i][j];
	        for (k=0; k<j; k++)
	            sum -= A[i][k]*A[k][j];
	        A[i][j] = sum;
	        dum = WrkBuf[i]*(double)fabs(sum);
	        if (dum >= fmax) {
	            imax = i;
	            fmax = dum;
	        }
	    }

	    if (j != imax) {
	        for (k=0; k<N; k++) {
	            dum = A[imax][k];
	            A[imax][k] = A[j][k];
	            A[j][k] = dum;
	        }
	        WrkBuf[imax] = WrkBuf[j];
	    }

	    Indx[j] = imax;

	    if (0 == A[j][j])
	        A[j][j] = 1e-20f;

	    if (j != N-1) {
	        dum = 1.f/A[j][j];
	        for (i=j+1; i<N; i++)
	            A[i][j] *= dum;
	    }
	} // end of loop over the columns

	return true;
	}


	/***********************************************************************
	Solve matrix equation where the matrix has passed the LU_Decomp
	routine.
	    Input:
	A[N][N] - output from LU_Decomp
	Indx    - output from LU_Decomp
	B - right side vector
	    Output:
	B - the equation solution
	*********************************************************************/
private	void LU_Solve(
	    double  A[][],
	    int    N,
	    int    Indx[],
	    double  B[]
	    )
	{
	int ii, i, j, LL;
	double sum;

	ii=-1;
	for (i=0; i<N; i++) {
	    LL = Indx[i];
	    sum = B[LL];
	    B[LL] = B[i];
	    if (ii != -1) {
	        for (j=ii; j<i; j++)
	            sum -= A[i][j]*B[j];
	    }
	    else if (sum != 0)
	        ii = i;
	    B[i] = sum;
	}

	for (i=N-1; i >= 0; i--) {
	    sum = B[i];
	    if (i < N-1) {
	        for (j=i+1; j<N; j++)
	            sum -= A[i][j]*B[j];
	    }
	    B[i] = sum/A[i][i];
	}
 }

}

	



