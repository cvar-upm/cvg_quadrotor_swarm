#include "tools.h"

#include <list>


float AngRango (float ang)
{
	if (ang > PI)
	{
		ang=ang-2*PI;
		AngRango(ang);
	}
	if (ang < -PI)
	{
		ang=2*PI+ang;
		AngRango(ang);
	}

	return ang;
}

ColumnVector Comp(ColumnVector ab,ColumnVector bc)		
{
	HomogTrans T_ab;
	HomogTrans T_bc;

	HomogTrans T_ac;
	Matrix ac(6,1);

	T_ab.DirectTrans(ab(1),ab(2),ab(3),ab(4),ab(5),ab(6));
	T_bc.DirectTrans(bc(1),bc(2),bc(3),bc(4),bc(5),bc(6));
	T_ac=T_ab*T_bc;
	
	ac << GetCoord(T_ac);

	return ac;
}

ColumnVector CompOdom(ColumnVector ab, ColumnVector bc)
{
	ColumnVector ac;
	ac(1)=bc(1)*cos(ab(4))*cos(ab(5))+bc(2)*(cos(ab(4))*sin(ab(5))*sin(ab(6))-sin(ab(4))*cos(ab(6)));
	ac(2)=bc(1)*sin(ab(4))*cos(ab(5))+bc(2)*(sin(ab(4))*sin(ab(5))*sin(ab(6))+cos(ab(4))*cos(ab(6)));
	ac(3)=-bc(1)*sin(ab(5))+bc(2)*cos(ab(5))*sin(ab(6));
	ac(4)=bc(4);
	ac(5)=0;
	ac(6)=0;

	return ac;
}


ColumnVector Inv(ColumnVector ab)
{
	HomogTrans t;

	t.InvTrans(ab(1),ab(2),ab(3),ab(4),ab(5),ab(6));


	/*double e_00=t.mat[0][0]; double e_01=t.mat[0][1]; double e_02=t.mat[0][2]; double e_03=t.mat[0][3];
	double e_10=t.mat[1][0]; double e_11=t.mat[1][1]; double e_12=t.mat[1][2]; double e_13=t.mat[1][3];
	double e_20=t.mat[2][0]; double e_21=t.mat[2][1]; double e_22=t.mat[2][2]; double e_23=t.mat[2][3];
	double e_30=t.mat[3][0]; double e_31=t.mat[3][1]; double e_32=t.mat[3][2]; double e_33=t.mat[3][3];*/

	ColumnVector inv(6);
	inv << GetCoord(t);
	return inv;


}

Matrix GetCoord (const HomogTrans& h)
{
	double x=h.mat[0][3];
	double y=h.mat[1][3];
	double z=h.mat[2][3];
	double nx=h.mat[0][0]; double ny=h.mat[1][0]; double nz=h.mat[2][0];
	double ox=h.mat[0][1]; double oy=h.mat[1][1]; double oz=h.mat[2][1];
	double ax=h.mat[0][2]; double ay=h.mat[1][2]; double az=h.mat[2][2];
	double roll=atan2(ny,nx);
	double pitch=atan2(-nz,nx*cos(roll)+ny*sin(roll));
	double yaw=atan2(ax*sin(roll)-ay*cos(roll), -ox*sin(roll)+oy*cos(roll));

	Real c[6]={x,y,z,roll,pitch,yaw};
	Matrix coord(6,1);
	coord << c;
	return coord;
}

Matrix J1(ColumnVector ab,ColumnVector bc, ColumnVector ac)   
{                
	Matrix j1(6,6);
	IdentityMatrix eye(3);
	Matrix zeros(3,3);
	zeros=0.0;
	j1.SubMatrix(1,3,1,3)=eye;
	j1.SubMatrix(4,6,1,3)=zeros;

	HomogTrans H1;
	H1.DirectTrans(ab(1),ab(2),ab(3),ab(4),ab(5),ab(6));
	HomogTrans H2;
	H2.DirectTrans(bc(1),bc(2),bc(3),bc(4),bc(5),bc(6));

	Matrix M(3,3);
	M.Row(1) << ab(2)-ac(2) << (ac(3)-ab(3))*cos(ab(4)) << H1.mat[1][3]*bc(2)-H1.mat[1][2]*bc(3);
	M.Row(2) << ac(1)-ab(1) << (ac(3)-ab(3))*sin(ab(4)) << H1.mat[2][3]*bc(2)-H1.mat[2][2]*bc(3);
	M.Row(3) << 0 << -bc(1)*cos(ab(5))-bc(2)*sin(ab(5))*sin(ab(6))-bc(3)*sin(ab(5))*cos(ab(6)) << H1.mat[3][3]*bc(2)-H1.mat[3][2]*bc(3);

	Matrix K1(3,3);
	K1.Row(1) << 1 << sin(ac(5))*sin(ac(4)-ab(4))/cos(ac(5)) << H2.mat[1][2]*sin(ac(6))+H2.mat[1][3]*cos(ac(6));
	K1.Row(2) << 0 << cos(ac(4)-ab(4)) << -cos(ab(5))*sin(ac(4)-ab(4));
	K1.Row(3) << 0 << sin(ac(4)-ab(4))/cos(ac(5)) << cos(ab(5))*cos(ac(4)-ab(4))/cos(ac(5));


	j1.SubMatrix(1,3,4,6)=M;
	j1.SubMatrix(4,6,4,6)=K1;

	
	return j1;
}

Matrix J1(ColumnVector ab,ColumnVector bc)
{
	Matrix j1(6,6);
	IdentityMatrix eye(3);
	Matrix zeros(3,3);
	zeros=0.0;
	j1.SubMatrix(1,3,1,3)=eye;
	j1.SubMatrix(4,6,1,3)=zeros;

	HomogTrans H1;
	H1.DirectTrans(ab(1),ab(2),ab(3),ab(4),ab(5),ab(6));
	HomogTrans H2;
	H2.DirectTrans(bc(1),bc(2),bc(3),bc(4),bc(5),bc(6));

	ColumnVector ac(6);
	ac << Comp(ab,bc);

	Matrix M(3,3);
	M.Row(1) << ab(2)-ac(2) << (ac(3)-ab(3))*cos(ab(4)) << H1.mat[1][3]*bc(2)-H1.mat[1][2]*bc(3);
	M.Row(2) << ac(1)-ab(1) << (ac(3)-ab(3))*sin(ab(4)) << H1.mat[2][3]*bc(2)-H1.mat[2][2]*bc(3);
	M.Row(3) << 0 << -bc(1)*cos(ab(5))-bc(2)*sin(ab(5))*sin(ab(6))-bc(3)*sin(ab(5))*cos(ab(6)) << H1.mat[3][3]*bc(2)-H1.mat[3][2]*bc(3);

	Matrix K1(3,3);
	K1.Row(1) << 1 << sin(ac(5))*sin(ac(4)-ab(4))/cos(ac(5)) << H2.mat[1][2]*sin(ac(6))+H2.mat[1][3]*cos(ac(6));
	K1.Row(2) << 0 << cos(ac(4)-ab(4)) << -cos(ab(5))*sin(ac(4)-ab(4));
	K1.Row(3) << 0 << sin(ac(4)-ab(4))/cos(ac(5)) << cos(ab(5))*cos(ac(4)-ab(4))/cos(ac(5));


	j1.SubMatrix(1,3,4,6)=M;
	j1.SubMatrix(4,6,4,6)=K1;

	return j1;

}

Matrix J1_n(ColumnVector ab,ColumnVector bc)
{
	Matrix j1(6,6);
	IdentityMatrix eye(3);
	Matrix zeros(3,3);
	zeros=0.0;
	j1.SubMatrix(1,3,1,3)=eye;
	j1.SubMatrix(4,6,1,3)=zeros;

	HomogTrans H1;
	H1.DirectTrans(ab(1),ab(2),ab(3),ab(4),ab(5),ab(6));
	HomogTrans H2;
	H2.DirectTrans(bc(1),bc(2),bc(3),bc(4),bc(5),bc(6));

	ColumnVector ac(6);
	ac << Comp(ab,bc);

	Matrix M(3,3);
	M.Row(1) << ab(2)-ac(2) << (ac(3)-ab(3))*cos(ab(4)) << H1.mat[0][2]*bc(2)-H1.mat[0][1]*bc(3);
	M.Row(2) << ac(1)-ab(1) << (ac(3)-ab(3))*sin(ab(4)) << H1.mat[1][2]*bc(2)-H1.mat[1][1]*bc(3);
	M.Row(3) << 0 << -bc(1)*cos(ab(5))-bc(2)*sin(ab(5))*sin(ab(6))-bc(3)*sin(ab(5))*cos(ab(6)) << H1.mat[2][2]*bc(2)-H1.mat[2][1]*bc(3);

	Matrix K1(3,3);
	K1.Row(1) << 1 << sin(ac(5))*sin(ac(4)-ab(4))/cos(ac(5)) << (H2.mat[0][1]*sin(ac(6))+H2.mat[0][2]*cos(ac(6)))/cos(ac(5));
	K1.Row(2) << 0 << cos(ac(4)-ab(4)) << -cos(ab(5))*sin(ac(4)-ab(4));
	K1.Row(3) << 0 << sin(ac(4)-ab(4))/cos(ac(5)) << cos(ab(5))*cos(ac(4)-ab(4))/cos(ac(5));


	j1.SubMatrix(1,3,4,6)=M;
	j1.SubMatrix(4,6,4,6)=K1;

	return j1;

}

Matrix J1Odom(ColumnVector ab,ColumnVector bc)   
{                
	Matrix j1(6,6);
	IdentityMatrix eye(3);
	Matrix zeros=0*eye;
	j1.SubMatrix(1,3,1,3)=eye;
	j1.SubMatrix(4,6,4,6)=eye;
	j1.SubMatrix(4,6,1,3)=zeros;

	Matrix M(3,3);
	M(1,1)=-cos(ab(5))*sin(ab(4))*bc(1)-(sin(ab(4))*sin(ab(5))*sin(ab(6))+cos(ab(4))*cos(ab(6)))*bc(2);
	M(1,2)=-bc(1)*sin(ab(4))*sin(ab(5))+sin(ab(4))*cos(ab(5))*sin(ab(6))*bc(2);
	M(1,3)=bc(2)*(cos(ab(4))*sin(ab(5))*cos(ab(6))+sin(ab(4)*sin(ab(6))));
	M(2,1)=bc(1)*cos(ab(4))*cos(ab(5))+bc(2)*(cos(ab(4))*sin(ab(5))*sin(ab(6))-sin(ab(4))*cos(ab(6)));
	M(2,2)=-bc(1)*sin(ab(4))*sin(ab(5))+bc(2)*sin(ab(4))*cos(ab(5))*sin(ab(6));
	M(2,3)=bc(2)*(sin(ab(4))*sin(ab(5))*cos(ab(6))-cos(ab(4))*sin(ab(6)));
	M(3,1)=0;
	M(3,2)=-bc(1)*cos(ab(5));
	M(3,3)=bc(2)*cos(ab(5))*cos(ab(6));

	j1.SubMatrix(1,3,4,6)=M;

	return j1;
}

Matrix J1_M (ColumnVector ab,ColumnVector bc)   
{                

	Matrix M(3,3);
	M(1,1)=-cos(ab(5))*sin(ab(4))*bc(1)-(sin(ab(4))*sin(ab(5))*sin(ab(6))+cos(ab(4))*cos(ab(6)))*bc(2);
	M(1,2)=-bc(1)*sin(ab(4))*sin(ab(5))+sin(ab(4))*cos(ab(5))*sin(ab(6))*bc(2);
	M(1,3)=bc(2)*(cos(ab(4))*sin(ab(5))*cos(ab(6))+sin(ab(4)*sin(ab(6))));
	M(2,1)=bc(1)*cos(ab(4))*cos(ab(5))+bc(2)*(cos(ab(4))*sin(ab(5))*sin(ab(6))-sin(ab(4))*cos(ab(6)));
	M(2,2)=-bc(1)*sin(ab(4))*sin(ab(5))+bc(2)*sin(ab(4))*cos(ab(5))*sin(ab(6));
	M(2,3)=bc(2)*(sin(ab(4))*sin(ab(5))*cos(ab(6))-cos(ab(4))*sin(ab(6)));
	M(3,1)=0;
	M(3,2)=-bc(1)*cos(ab(5));
	M(3,3)=bc(2)*cos(ab(5))*cos(ab(6));

	return M;
}


Matrix J2(ColumnVector ab, ColumnVector bc, ColumnVector ac) 
{
	Matrix j2(6,6);
	IdentityMatrix eye(3);
	Matrix zeros(3,3);
	zeros=0.0;
	j2.SubMatrix(1,3,4,6)=zeros;
	j2.SubMatrix(4,6,1,3)=zeros;

	Matrix R1(3,3);
	R1(1,1)=cos(ab(4))*cos(ab(5));
	R1(1,2)=cos(ab(4))*sin(ab(5))*sin(ab(6))-sin(ab(4))*cos(ab(6));
	R1(1,3)=cos(ab(4))*sin(ab(5))*cos(ab(6))+sin(ab(4))*sin(ab(6));
	R1(2,1)=sin(ab(4))*cos(ab(5));
	R1(2,2)=sin(ab(4))*sin(ab(5))*sin(ab(6))+cos(ab(4))*cos(ab(6));
	R1(2,3)=sin(ab(4))*sin(ab(5))*cos(ab(6))-cos(ab(4))*sin(ab(6));
	R1(3,1)=-sin(ab(5));
	R1(3,2)=cos(ab(5))*sin(ab(6));
	R1(3,3)=cos(ab(5))*cos(ab(6));

	HomogTrans H1;
	H1.DirectTrans(ab(1),ab(2),ab(3),ab(4),ab(5),ab(6));

	Matrix K2(3,3);
	K2.Row(1) << -cos(bc(5))*cos(ac(6)-bc(6))/cos(ac(5)) << sin(ac(6)-bc(6)) << 0;
	K2.Row(2) << -cos(ab(5))*sin(ac(6)-bc(6)) << cos(ac(6)-bc(6)) << 0;
	K2.Row(3) << H1.mat[1][3]*cos(ac(4)) << sin (ac(5))*sin (ac(6)-bc(6))/ cos(ac(5)) << 1 ;



	j2.SubMatrix(1,3,1,3)=R1;
	j2.SubMatrix(4,6,4,6)=K2;
	return j2;

}


Matrix J2(ColumnVector ab, ColumnVector bc)
{
	Matrix j2(6,6);
	IdentityMatrix eye(3);
	Matrix zeros(3,3);
	zeros=0.0;
	j2.SubMatrix(1,3,4,6)=zeros;
	j2.SubMatrix(4,6,1,3)=zeros;

	Matrix R1(3,3);
	R1(1,1)=cos(ab(4))*cos(ab(5));
	R1(1,2)=cos(ab(4))*sin(ab(5))*sin(ab(6))-sin(ab(4))*cos(ab(6));
	R1(1,3)=cos(ab(4))*sin(ab(5))*cos(ab(6))+sin(ab(4))*sin(ab(6));
	R1(2,1)=sin(ab(4))*cos(ab(5));
	R1(2,2)=sin(ab(4))*sin(ab(5))*sin(ab(6))+cos(ab(4))*cos(ab(6));
	R1(2,3)=sin(ab(4))*sin(ab(5))*cos(ab(6))-cos(ab(4))*sin(ab(6));
	R1(3,1)=-sin(ab(5));
	R1(3,2)=cos(ab(5))*sin(ab(6));
	R1(3,3)=cos(ab(5))*cos(ab(6));

	HomogTrans H1;
	H1.DirectTrans(ab(1),ab(2),ab(3),ab(4),ab(5),ab(6));

	ColumnVector ac(6);
	ac << Comp(ab,bc);

	Matrix K2(3,3);
	K2.Row(1) << cos(bc(5))*cos(ac(6)-bc(6))/cos(ac(5)) << sin(ac(6)-bc(6)) << 0; //-?
	K2.Row(2) << -cos(bc(5))*sin(ac(6)-bc(6)) << cos(ac(6)-bc(6)) << 0;
	K2.Row(3) << (H1.mat[1][3]*cos(ac(4)) + H1.mat[2][3]*sin(ac(4)))/cos(ac(5)) << sin (ac(5))*sin (ac(6)-bc(6))/ cos(ac(5)) << 1 ;

	j2.SubMatrix(1,3,1,3)=R1;
	j2.SubMatrix(4,6,4,6)=K2;
	return j2;

}

Matrix J2_n(ColumnVector ab, ColumnVector bc)
{
	Matrix j2(6,6);
	IdentityMatrix eye(3);
	Matrix zeros(3,3);
	zeros=0.0;
	j2.SubMatrix(1,3,4,6)=zeros;
	j2.SubMatrix(4,6,1,3)=zeros;

	Matrix R1(3,3);
	R1(1,1)=cos(ab(4))*cos(ab(5));
	R1(1,2)=cos(ab(4))*sin(ab(5))*sin(ab(6))-sin(ab(4))*cos(ab(6));
	R1(1,3)=cos(ab(4))*sin(ab(5))*cos(ab(6))+sin(ab(4))*sin(ab(6));
	R1(2,1)=sin(ab(4))*cos(ab(5));
	R1(2,2)=sin(ab(4))*sin(ab(5))*sin(ab(6))+cos(ab(4))*cos(ab(6));
	R1(2,3)=sin(ab(4))*sin(ab(5))*cos(ab(6))-cos(ab(4))*sin(ab(6));
	R1(3,1)=-sin(ab(5));
	R1(3,2)=cos(ab(5))*sin(ab(6));
	R1(3,3)=cos(ab(5))*cos(ab(6));

	HomogTrans H1;
	H1.DirectTrans(ab(1),ab(2),ab(3),ab(4),ab(5),ab(6));

	ColumnVector ac(6);
	ac << Comp(ab,bc);

	Matrix K2(3,3);
	K2.Row(1) << cos(bc(5))*cos(ac(6)-bc(6))/cos(ac(5)) << sin(ac(6)-bc(6)) << 0; //-?
	K2.Row(2) << -cos(bc(5))*sin(ac(6)-bc(6)) << cos(ac(6)-bc(6)) << 0;
	K2.Row(3) << (H1.mat[0][2]*cos(ac(4)) + H1.mat[1][2]*sin(ac(4)))/cos(ac(5)) << sin (ac(5))*sin (ac(6)-bc(6))/ cos(ac(5)) << 1 ;

	j2.SubMatrix(1,3,1,3)=R1;
	j2.SubMatrix(4,6,4,6)=K2;
	return j2;

}


Matrix J2Odom(ColumnVector ab,ColumnVector bc)   
{                
	Matrix j2(6,6);
	IdentityMatrix eye(3);
	Matrix zeros=0*eye;
	
	
	j2.SubMatrix(1,3,4,6)=zeros;
	j2.SubMatrix(4,6,1,3)=zeros;

	Matrix M1(3,3);
	M1(1,1)=cos(ab(4))*cos(ab(5));
	M1(1,2)=cos(ab(4))*sin(ab(5))*sin(ab(6))-sin(ab(4))*cos(ab(6));
	M1(1,3)=0;
	M1(2,1)=sin(ab(4))*cos(ab(5));
	M1(2,2)=sin(ab(4))*sin(ab(5))*sin(ab(6))+cos(ab(4))*cos(ab(6));
	M1(2,3)=0;
	M1(3,1)=-sin(ab(5));
	M1(3,2)=cos(ab(5))*sin(ab(6));
	M1(3,3)=0;

	Matrix M2(3,3);
	M2.Row(1) << 1 << 0 << 0;
	M2.Row(2) << 0 << 0 << 0;
	M2.Row(3) << 0 << 0 << 0;

	j2.SubMatrix(1,3,4,6)=M1;
	j2.SubMatrix(4,6,4,6)=M2;
	return j2;
}


Matrix J2_A(ColumnVector ab,ColumnVector bc)
{
	Matrix A(3,2);
	A(1,1)=cos(ab(4))*cos(ab(5));
	A(1,2)=cos(ab(4))*sin(ab(5))*sin(ab(6))-sin(ab(4))*cos(ab(6));
	A(2,1)=sin(ab(4))*cos(ab(5));
	A(2,2)=sin(ab(4))*sin(ab(5))*sin(ab(6))+cos(ab(4))*cos(ab(6));
	A(3,1)=-sin(ab(5));
	A(3,2)=cos(ab(5))*sin(ab(6));

	return A;

}

Matrix JInv(ColumnVector v)
{	
	double x=v(1);
	double y=v(2);
	double z=v(3);
	double roll=v(4);
	double pitch=v(5);
	double yaw=v(6);
	
	HomogTrans h;	
	h.DirectTrans(x,y,z,roll,pitch,yaw);

	double nx=h.mat[0][0];	double ox=h.mat[0][1];	double ax=h.mat[0][2];
	double ny=h.mat[1][0];	double oy=h.mat[1][1];	double ay=h.mat[1][2];
	double nz=h.mat[2][0];	double oz=h.mat[2][1];	double az=h.mat[2][2];

	Matrix j(6,6);
	IdentityMatrix eye(3);
	Matrix zeros=0*eye;
	Matrix A(3,3);

	A.Row(1) << cos(roll)*cos(pitch) << sin(roll)*cos(pitch) << -sin(pitch);
	A.Row(2) << cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw) << sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw) << cos(pitch)*sin(yaw);
	A.Row(3) << cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw) << sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw) << cos(pitch)*cos(yaw);

	j.SubMatrix(1,3,1,3)=zeros;   //check, the other way around, A is the rotation
	j.SubMatrix(4,6,1,3)=A;

	Matrix N(3,3);
	N.Row(1) << ny*x-nx*y << -nz*x*cos(roll)-nz*y*sin(roll) << 0;
	N.Row(2) << oy*x-ox*y << -oz*x*cos(roll)-oz*y*sin(roll)+z*sin(pitch)*sin(yaw) << z;  //FIXME, inverse coord
	N.Row(3) << ay*x-ax*y << -az*x*cos(roll)-az*y*sin(roll)+z*sin(pitch)*cos(yaw) << -y;

	Matrix Q(3,3);
	Q.Row(1) << -az/(1-ax*ax) << -ay*cos(roll)/(1-ax*ax) << nx*ax/(1-ax*ax);
	Q.Row(2) << ay/sqrt(1-ax*ax) << -az*cos(roll)/sqrt(1-ax*ax) << ox/sqrt(1-ax*ax);
	Q.Row(3) << az*ax/(1-ax*ax) << -ox*cos(yaw)/(1-ax*ax) << -nx/(1-ax*ax);

	j.SubMatrix(1,3,4,6)=N;
	j.SubMatrix(4,6,4,6)=Q;

	return j;


}

Matrix eye(int n)
{
	Matrix identity(n,n);
	for(int i=0;i<n;i++)
		for (int j=0;j<n; j++)
		{
			if(j==i)
				identity(i,j)=1;
			else
				identity(i,j)=0;
		}
	return identity;
}


Matrix InvTrans(Matrix x_ab)          //Calculates the inverse transformation
{
	Matrix x_ba(3,1);

	float sen_alfa_ab=sin(x_ab(2,0));
	float cos_alfa_ab=cos(x_ab(2,0));

	x_ba(0,0)= -x_ab(0,0)*cos_alfa_ab-x_ab(1,0)*sen_alfa_ab;
	x_ba(1,0)= x_ab(0,0)*sen_alfa_ab-x_ab(1,0)*cos_alfa_ab;
	x_ba(2,0)= -x_ab(2,0);

	return x_ba;

}


