
#include "Kalman_Loc2.h"
#include "tools.h"
#include "drlgl/gltools.h"
#include "drlgl/glscene.h"

using namespace std;

void OnDraw(void);
void OnTimer(int value); //esta funcion sera llamada cuando transcurra una temporizacion
void OnKeyboardDown(unsigned char key, int x, int y); //cuando se pulse una tecla
void OnMouseMove(int x,int y);
void OnMouseClick(int button,int state, int x,int y);


Kalman_Loc loc;

vector<vector<float> > path;
vector<vector<float> > odom_path;
vector<vector<float> > real_path;

ColumnVector real_pos(6); 
ColumnVector odom_pos(6); 

GLScene scene;

int main(int argc, char* argv[])
{
	real_pos= 0.0;
	odom_pos= 0.0;

	vector<float> initial_pose;
	initial_pose.push_back(0); initial_pose.push_back(0); initial_pose.push_back(0);
	initial_pose.push_back(0); initial_pose.push_back(0); initial_pose.push_back(0);
	
	path.push_back(initial_pose);
	odom_path.push_back(initial_pose);
	real_path.push_back(initial_pose);
	
	loc.initialization();

	//GL Initialization stuff
	glutInit(&argc, argv);
	glutInitWindowSize(800,600);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutCreateWindow("GL");
	glutDisplayFunc(OnDraw);
	glutMotionFunc(OnMouseMove);
	glutMouseFunc(OnMouseClick);
	glutKeyboardFunc(OnKeyboardDown);
	
	scene.init();

	glutMainLoop();

	return 0;
}

void OnDraw(void)
{

	scene.draw();
	glutSwapBuffers();

	for (int i=0; i<path.size();i++)
	{
		double x = path[i][0]; 
		double y = path[i][1]; 
		double z = path[i][2];
		
		GLTools::Color(CYAN);
		glPointSize(5);
		glBegin(GL_POINTS);
			glVertex3f(x,y,z);
		glEnd();
	
	}
	
	for (int i=0; i<odom_path.size();i++)
	{
		double x = odom_path[i][0]; 
		double y = odom_path[i][1]; 
		double z = odom_path[i][2];
		
		GLTools::Color(GREEN);
		glPointSize(5);
		glBegin(GL_POINTS);
			glVertex3f(x,y,z);
		glEnd();
	
	}
	
	for (int i=0; i<real_path.size();i++)
	{
		double x = real_path[i][0]; 
		double y = real_path[i][1]; 
		double z = real_path[i][2];
		
		GLTools::Color(RED);
		glPointSize(5);
		glBegin(GL_POINTS);
			glVertex3f(x,y,z);
		glEnd();
	
	}
	
	
	glutSwapBuffers();
}

vector<Observation3D> getObs()
{
	vector<Observation3D> observations;
	
	for (int i=0; i< loc.map.size(); i++)
	{
		Landmark3D l = loc.map[i];
		int id = l.id;
		//cout << "id " << id << endl;
		ColumnVector lm(6);
		lm(1) = l.x; lm(2) = l.y; lm(3) = l.z;
		lm(4) = l.roll; lm(5) = l.pitch; lm(6) = l.yaw;
		
		ColumnVector pos = real_pos;
		Matrix i_pos = Inv(pos);
		//cout << "ipos " << i_pos(1,1) << " " << i_pos(2,1) << " " << i_pos(3,1) << " " << i_pos(4,1)*RAD2DEG << " " << i_pos(5,1)*RAD2DEG << " " << i_pos(6,1)*RAD2DEG << endl;
		ColumnVector ob = Comp(i_pos,lm);
	
		double noise_obs = 0.0*(rand()/(float)RAND_MAX);
		double noise_obs_ang = 0.0*DEG2RAD*(rand()/(float)RAND_MAX);
	
		Observation3D obs(id,ob(1)+noise_obs,ob(2)+noise_obs,ob(3)+noise_obs,ob(4)+noise_obs_ang,ob(5)+noise_obs_ang,ob(6)+noise_obs_ang);
		//cout << "obs " << obs.id << " " << obs.x << " " << obs.y << " " << obs.z << " " << obs.roll*RAD2DEG << " " << obs.pitch*RAD2DEG << " " << obs.yaw*RAD2DEG << endl;
	
		observations.push_back(obs);
	}
	
	return observations;
}

void OnTimer(int value)
{
	//glutTimerFunc(1000,OnTimer,0); //set again the timer
	
	
}


void OnKeyboardDown(unsigned char key, int x, int y)
{
	if (key == 'a')
   {
   	double noise_odom = 0.5*(rand()/(float)RAND_MAX);
		double inc_x = 4;
		double noise_odom_ang = 0*DEG2RAD*(rand()/(float)RAND_MAX);
		double inc_roll = 10*DEG2RAD;
		
		double inc_pitch = 10*DEG2RAD;
		double inc_yaw = 10*DEG2RAD;
		
		ColumnVector inc_odom_real(6);
		inc_odom_real = 0.0;
		inc_odom_real(1) = inc_x;
		inc_odom_real(4) = inc_roll;
		inc_odom_real(5) = inc_pitch;
		inc_odom_real(6) = inc_yaw;
		real_pos = Comp(real_pos,inc_odom_real);
		vector<float> real_pos_v;
		real_pos_v.push_back(real_pos(1)); real_pos_v.push_back(real_pos(2)); real_pos_v.push_back(real_pos(3));
		real_pos_v.push_back(real_pos(4)); real_pos_v.push_back(real_pos(5)); real_pos_v.push_back(real_pos(6));
		real_path.push_back(real_pos_v);
		
		ColumnVector inc_odom(6);
		inc_odom = 0.0;
		inc_odom(1) = inc_x+noise_odom;
		inc_odom(4) = inc_roll+noise_odom_ang;
		inc_odom(5) = inc_pitch+noise_odom_ang;
		inc_odom(6) = inc_yaw+noise_odom_ang;
		odom_pos = Comp(odom_pos,inc_odom);
		vector<float> odom_pos_v;
		odom_pos_v.push_back(odom_pos(1)); odom_pos_v.push_back(odom_pos(2)); odom_pos_v.push_back(odom_pos(3));
		odom_pos_v.push_back(odom_pos(4)); odom_pos_v.push_back(odom_pos(5)); odom_pos_v.push_back(odom_pos(6));
		odom_path.push_back(odom_pos_v);
	
		loc.KalmanPredict(odom_pos(1),odom_pos(2),odom_pos(3),odom_pos(4),odom_pos(5),odom_pos(6));
		cout << "predict " << loc.x << " " << loc.y << " " << loc.z << " " << loc.roll*RAD2DEG << " " << loc.pitch*RAD2DEG << " " << loc.yaw*RAD2DEG << endl;
		vector<Observation3D> obs = getObs();
		loc.KalmanUpdate(obs);
		cout << "update " << loc.x << " " << loc.y << " " << loc.z << " " << loc.roll*RAD2DEG << " " << loc.pitch*RAD2DEG << " " << loc.yaw*RAD2DEG << endl;
		vector<float> corrected_pose;
		corrected_pose.push_back(loc.x); corrected_pose.push_back(loc.y); corrected_pose.push_back(loc.z);
		corrected_pose.push_back(loc.roll); corrected_pose.push_back(loc.pitch); corrected_pose.push_back(loc.yaw);
	
		path.push_back(corrected_pose);
	
		glutPostRedisplay();
        
   }

	scene.keyDown(key);
	glutPostRedisplay();

}


void OnMouseClick(int b,int state, int x,int y)
{
	bool down=(state==GLUT_DOWN);
	int button;
	if(b==GLUT_LEFT_BUTTON)
		button=MOUSE_LEFT_BUTTON;
	if(b==GLUT_RIGHT_BUTTON)
		button=MOUSE_RIGHT_BUTTON;

	int specialKey = glutGetModifiers();
	bool ctrlKey= (specialKey & GLUT_ACTIVE_CTRL)?true:false;
	bool sKey= specialKey&GLUT_ACTIVE_SHIFT ;

	scene.mouseButton(x,y,b,down,sKey,ctrlKey);
	glutPostRedisplay();
}

void OnMouseMove(int x,int y)
{
	scene.mouseMove(x,y);
	glutPostRedisplay();
}

