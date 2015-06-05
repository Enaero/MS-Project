#ifndef __HELPERS_H__
#define __HELPERS_H__

void multiply_4x4_4x1(dReal* result, const dReal* M, const dReal* v)
{
	for (int i = 0; i < 4; ++i)
		result[i] = v[0]*M[i*4] + v[1]*M[i*4 + 1] + v[2]*M[i*4 + 2] + v[3]*M[i*4 + 3];
}

void multiply_4x1_1x1(dReal* result, const dReal a, const dReal* v)
{
	for (int i = 0; i < 4; ++i)
		result[i] = a*v[i];
}

void set_4x1(dReal* result, dReal a, dReal b, dReal c, dReal d)
{
	result[0] = a;
	result[1] = b;
	result[2] = c;
	result[3] = d;
}

void cross_4x1(dReal* result, const dReal* a, const dReal* b)
{
	result[0] = a[1]*b[2] - a[2]*b[1];
	result[1] = a[2]*b[0] - a[0]*b[2];
	result[2] = a[0]*b[1] - a[1]*b[0];
	result[3] = a[3];
}
void set_4x1(dReal* result, const dReal* input)
{
	result[0] = input[0];
	result[1] = input[1];
	result[2] = input[2];
	result[3] = input[3];
}
void transpose_4x4(dReal* M)
{
	dMatrix4 A;
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			A[j*4+i] = M[i*4+j];
		}
	}

	for (int i = 0; i < 16; ++i)
		M[i] = A[i];
}

void set_identity_4x4(dReal* M)
{
	for (int i = 0; i < 16; ++i)
	{
		M[i] = 0;
	}
	for (int i = 0; i < 4; ++i)
	{
		M[4*i + i] = 1;
	}
}

void rotate_vector(dVector4 v, dReal angle, dReal x, dReal y, dReal z)
{
	dMatrix4 R;
	dVector4 temp;
	set_identity_4x4(R);
	dRFromAxisAndAngle(R, x, y, z, angle);

	set_4x1(temp, v[0], v[1], v[2], v[3]);
	multiply_4x4_4x1(v, R, temp);
}

void add_4x1(dReal* result, const dReal* A, const dReal* B)
{
	for (int i = 0; i < 4; ++i)
		result[i] = A[i] + B[i];
}
void subtract_4x1(dReal* result, const dReal* A, const dReal* B)
{
	for (int i = 0; i < 4; ++i)
	{
		result[i] = A[i]-B[i];
	}
}

dReal magnitude_4x1(const dReal* v)
{
	return dSqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

void normalize_4x1(dReal* v)
{
	dReal m = magnitude_4x1(v);
	if (m == 0.0)
	{
		return;
	}
	v[0] /= m;
	v[1] /= m;
	v[2] /= m;
	v[3] /= m;
}

dReal dot_4x1 (const dVector4 a, const dVector4 b)
{
	return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3];
}

void multiply_3x3_3x3(dMatrix3 result, const dMatrix3 a, const dMatrix3 b)
{
	dReal tmpA[3][3];
	dReal tmpB[3][3];

	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
		{
			tmpA[i][j] = a[i*4+j];
			tmpB[i][j] = b[i*4+j];
		}

	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			dReal sum = 0;
			for (int k = 0; k < 3; ++k)
				sum += tmpA[i][k]*tmpB[k][j];
			result[i*4+j] = sum;
		}
	}
}

void multiply_4x4_4x4(dMatrix4 result, const dMatrix4 a, const dMatrix4 b)
{
	dReal tmpA[4][4];
	dReal tmpB[4][4];

	for (int i = 0; i < 4; ++i)
		for (int j = 0; j < 4; ++j)
		{
			tmpA[i][j] = a[i*4+j];
			tmpB[i][j] = b[i*4+j];
		}

	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			dReal sum = 0;
			for (int k = 0; k < 4; ++k)
				sum += tmpA[i][k]*tmpB[k][j];
			result[i*4+j] = sum;
		}
	}
}

bool invert_3x3 (dMatrix3 result, const dMatrix3 m)
{
	dReal tmp[3][3]; //row#, col#

	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
		{
			tmp[i][j] = m[i*4 + j];
		}

	dReal det = tmp[0][0]*(tmp[1][1]*tmp[2][2]-tmp[2][1]*tmp[1][2])
				-tmp[0][1]*(tmp[1][0]*tmp[2][2]-tmp[2][0]*tmp[1][2])
				+tmp[0][2]*(tmp[1][0]*tmp[2][1]-tmp[1][1]*tmp[2][0]);

	if (det == 0)
		return false;

	dReal transpose[3][3];

	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			transpose[i][j] = tmp[j][i];

	dReal cofactors[9];

	for (int x = 0; x < 3; ++x)
	{
		for (int y = 0; y < 3; ++y)
		{
			int i = (y+3+1)%3;
			int j = (x+3+1)%3;
			cofactors[i*3+j] = tmp[i][j]*tmp[(i+3+1)%3][(j+3+1)%3]-tmp[i][(j+3+1)%3]*tmp[(i+3+1)%3][j];
			cofactors[i*3+j] /= det;

			//cout << cofactors[i*3+j] << ' ';
		}
		//cout << endl;
	}
	//cout << endl;

	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			result[i*4+j]= cofactors[(j+1+3)%3*3+(i+1+3)%3];
	/*
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			cout << result[i*4+j] << ' ';
		}
		cout << endl;
	}*/
	return true;
}

void RtoAngleAxis(dReal& angle, dVector3 axis, const dMatrix3 A)
{
	//http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToAngle/

	angle = acos((A[4*0+0] + A[4*1+1] + A[4*2+2] - 1.0)/2.0);

	if (angle == 0.0)
	{
		axis[0] = 0;
		axis[1] = 0;
		axis[2] = 0;
		axis[3] = 0;
		return;
	}

	dReal x = (A[4*2+1] - A[4*1+2]) / sqrt( pow(A[4*2+1] - A[4*1+2], 2) + pow(A[4*0+2] - A[4*2+0], 2) + pow(A[4*1+0] - A[4*0+1], 2) );
	dReal y = (A[4*0+2] - A[4*2+0]) / sqrt( pow(A[4*2+1] - A[4*1+2], 2) + pow(A[4*0+2] - A[4*2+0], 2) + pow(A[4*1+0] - A[4*0+1], 2) );
	dReal z = (A[4*1+0] - A[4*0+1]) / sqrt( pow(A[4*2+1] - A[4*1+2], 2) + pow(A[4*0+2] - A[4*2+0], 2) + pow(A[4*1+0] - A[4*0+1], 2) );

	axis[0] = x;
	axis[1] = y;
	axis[2] = z;
	axis[3] = 0;
}

void calculate_angle_axis(dReal& angle, dVector3 axis, dBodyID body, const dMatrix3 natural_orientation)
{
	//X C = D
	//C = DX-1
	//D-1C = = X-1
	//X = (D-1C)-1
	// X = C-1D
	const dReal* current_orientation = dBodyGetRotation(body);

	dMatrix3 Rinv;

	invert_3x3(Rinv, current_orientation);

	dMatrix3 A;

	multiply_3x3_3x3(A, Rinv, natural_orientation);

	RtoAngleAxis(angle, axis, A);
}

dReal angleBetween_4x1(const dReal* a, const dReal* b)
{
	dReal cosA = dot_4x1(a, b)/(magnitude_4x1(a)*magnitude_4x1(b));

	if (cosA > 0.9999)
		return 0;
	else if (cosA < -0.9999)
		return 3.141592;
	else
		return acos(cosA);
}

void AngleAxistoR(dMatrix3 R, dReal angle, const dVector3 axis)
{
	//http://en.wikipedia.org/wiki/Rotation_matrix#Conversion_from_and_to_axis-angle
	dVector3 copy;
	for (int i = 0; i < 3; ++i)
		copy[i] = axis[i];

	copy[3] = 0;

	normalize_4x1(copy);

	R[4*0 + 0] = cos(angle) + copy[0]*copy[0]*(1.0-cos(angle));
	R[4*0 + 1] = copy[0]*copy[1]*(1-cos(angle)) - copy[2]*sin(angle);
	R[4*0 + 2] = copy[0]*copy[2]*(1-cos(angle)) + copy[1]*sin(angle);
	R[4*0 + 3] = 0;

	R[4*1 + 0] = copy[1]*copy[0]*(1.0-cos(angle)) + copy[2]*sin(angle);
	R[4*1 + 1] = cos(angle) + copy[1]*copy[1]*(1.0-cos(angle));
	R[4*1 + 2] = copy[1]*copy[2]*(1-cos(angle)) - copy[0]*sin(angle);
	R[4*1 + 3] = 0;

	R[4*2 + 0] = copy[2]*copy[0]*(1-cos(angle)) - copy[1]*sin(angle);
	R[4*2 + 1] = copy[2]*copy[1]*(1.0-cos(angle)) + copy[0]*sin(angle);
	R[4*2 + 2] = cos(angle) + copy[2]*copy[2]*(1.0-cos(angle));
	R[4*2 + 3] = 0;

}

bool parallel(const dReal* a, const dReal* b)
{
	dVector4 A, B;
	set_4x1(A, a);
	set_4x1(B, b);

	normalize_4x1(A);
	normalize_4x1(B);

	for (int i = 0; i < 3; ++i)
		if (A[i] != B[i])
			return false;
	return true;
}
#endif