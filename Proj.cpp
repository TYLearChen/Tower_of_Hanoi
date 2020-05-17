#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <time.h>
#include <math.h>

#include "glut.h"

using namespace std;

#define PI			3.141592654  // Prime
#define WIN_POSX    400
#define WIN_POSY    400
#define WIN_WIDTH   400
#define WIN_HEIGHT  300

struct pos
{
	GLdouble x = 0;
	GLdouble y = 0;
	GLdouble z = 0;
};

GLfloat M[16];
pos light = { 2000, 2000, 2000 };
double t_prev;                   // previous time elapsed
const double exponent = 7;
const double ceiling = 120 + 160;
const double ground = 0;//-120;
double theta, phi = 0, psi;			 // rotation angles of robot, lower-and-upper arm, upper arm respectivley
pos org = { 0, ground, 120 };
pos top_left = { 0, ceiling, 120 };
pos top_mid  = { 0, ceiling, 0 };
pos top_right= { 0, ceiling, -120 };
pos bottom_left  = { 0, ground, 120 };
pos bottom_mid   = { 0, ground, 0 };
pos bottom_right = { 0, ground, -120 };
pos claw_pos;
pos disk_pos[3];
int cur_scene = 0;
const double sec_per_act = 2;
const double hight_disk = 20;
const double hight_lower_arm = 60;
const double hight_upper_arm = 30;
const double hight_total_claw = (30 + 60) * cos(phi) + 40;
float mo[16] = { 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };
float* gsrc_getmo(){ return mo; }

void cube()
// draw a standard 2 x 2 x 2 cube whose center is at (0, 1, 0)
{

    /* The use of glPushMatrix and glPopMatrix here protects the glRotatef from
	   exercising its influence on the relative transformation beyond this function */
    glPushMatrix ( );

	glTranslatef  (0, 1, 0);
	glutSolidCube (2);

    glPopMatrix  ( );

}

void draw_disk(int radius, int disk_hight, double colour[3])
{
	GLUquadricObj* obj[3];
	for (int i = 0; i < 3; i++)
		obj[i] = gluNewQuadric();

	glPushMatrix();

		glColor3f(colour[0] / 255, colour[1] / 255, colour[2] / 255);
		glRotatef(-90.0, 1.0, 0.0, 0.0);   // rotate about x axis by -90 deg.

		// draws a hollow cylinder along the z axis with base on x-y axis, origin at (0, 0, 0) 
		gluCylinder(obj[0], radius, radius, disk_hight, 10, 10);
		// 10 x 10 controls the sampling

		// draw a solid disk to cover the base 
		gluDisk(obj[1], 0, radius, 10, 10);

		// draw a solid disk to cover the top
		glPushMatrix();
			glTranslatef(0, 0, disk_hight);
			gluDisk(obj[2], 0, radius, 10, 10);
		glPopMatrix();

	glPopMatrix();
}

void draw_base(bool shadow = false)
// draw base : blue cylinder of radius 30 height 40  x-z as base
{
	GLUquadricObj* obj[3];
	for (int i = 0; i < 3; i++)
		obj[i] = gluNewQuadric();

	glPushMatrix ( );			
	if (shadow)
		glColor3f(0, 0, 0);
	else
		glColor3f (0.0, 0.0, 1.0);				   
	glRotatef (-90.0, 1.0, 0.0, 0.0);   // rotate about x axis by -90 deg.

    // draws a hollow cylinder along the z axis with base on x-y axis, origin at (0, 0, 0) 
	gluCylinder (obj[0], 30, 30, 40, 10, 10);
	// base radius 30  
	// top  radius 30 
	// height 40
    // 10 x 10 controls the sampling
	 
	// draw a solid disk to cover the base 
	gluDisk (obj[1], 0, 30, 10, 10);

	// draw a solid disk to cover the top
    glPushMatrix ( );
    glTranslatef (0, 0, 40);

	gluDisk (obj[2], 0, 30, 10, 10);
    glPopMatrix ( );

    glPopMatrix ( );
}

void draw_lower_arm(bool shadow = false)
{
	glPushMatrix ( );					// draw lower robotic arm : green box of dimension 15 x 70 x 15
	if (shadow)
		glColor3f(0, 0, 0);
	else
		glColor3f (0.0, 1.0, 0.0);			
	glScalef (7.5, hight_lower_arm / 2, 7.5);
	cube ( ); 
    glPopMatrix ( );
}

void draw_upper_arm(bool shadow = false)
{
    glPushMatrix ( );					// draw upper robotic arm : yellow box of dimension 15 x 40 x 15
	if (shadow)
		glColor3f(0, 0, 0);
	else
		glColor3f (1.0, 0.65, 0.0);			
    glScalef (7.5, hight_upper_arm / 2, 7.5);
	cube ( );
    glPopMatrix ( );
}

void draw_claw(bool shadow = false)
{
	glScalef(1, -1, 1);
	draw_base(shadow);

	glTranslatef(0.0, 40, 0.0);		// M_(lower arm to base)
	for (int i = 0; i < 4; i++)		// Draw 4 Arms
	{
		glRotatef(90 * i, 0, 1, 0);

		glPushMatrix();

		glTranslatef(-20.0, 0, 0.0);		// M_(lower arm to base)

		glRotatef(phi, 0.0, 0.0, 1.0);      // rotate upper and lower arm by phi degrees

		draw_lower_arm(shadow);

		glTranslatef(0.0, hight_lower_arm, 0.0);      // M_(upper arm to lower arm) 

		psi = -(2 * phi);

		glRotatef(psi, 0.0, 0.0, 1.0);      // rotate upper arm by psi degrees

		draw_upper_arm(shadow);

		glPopMatrix();
	}
}

void opClaw(char mode, double t)
{
	const double travel = sec_per_act;
	const double angle = 45;

	if (mode == 'o')	// Open
	{
		phi = angle * 0.3 + angle * 0.7 * pow(sin(PI * t / (2 * travel)), exponent);	// Deceleration
		cout << "opClaw : Open " << "Phi : " << phi << endl;
	}
	else if (mode == 'c') // Close
	{
		phi = angle * (1 - t * 0.7 / travel);
		cout << "opClaw : Close " << "Phi : " << phi << endl;
	}
}

void moveClaw(char axis, double start, double end, double t)
{
	double t_total = sec_per_act;
	double a;

	if (axis == 'x')
	{
		
	}
	else if (axis == 'y')
	{
		a = 2 * (end - start) / (t_total * t_total);
		claw_pos.y = start + 0.5 * a * pow(t, 2);
		cout << "claw_pos.y : " << claw_pos.y << endl;
	}
	else if (axis == 'z')
	{
		claw_pos.z = start + (end - start) * pow((1 - cos(t/t_total * PI)) / 2, exponent);
		cout << "claw_pos.z : " << claw_pos.z << endl;
	}
}

void moveBaseDisk(int disk, char axis, double start, double end, double t)
{
	double t_total = sec_per_act;
	double a = 98 * 5;

	if (axis == 'x')
	{

	}
	else if (axis == 'y')
	{
		a = 2 * (end - start) / (t_total * t_total);
		disk_pos[disk].y = start + 0.5 * a * pow(t, 2);
	}
	else if (axis == 'z')
	{
		disk_pos[disk].z = start + (end - start) * pow((1 - cos(t / t_total * PI)) / 2, exponent);
	}
}

void drawscene()
{
	//////////////////////////////////////////////////////////////////
	// 
	// Setup perspective projection and the rotation
	// 
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport); // viewport is by default the display window
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45, double(viewport[2]) / viewport[3], 0.1, 1000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(500, 300, 000, 0, 100, 0, 0, 1, 0); //0 0 400
	glMultMatrixf(gsrc_getmo());  // get the rotation matrix from the rotation user-interface
  //
  //////////////////////////////////////////////////////////////////


  /*  Enable Z buffer method for visibility determination. */
  //  Z buffer code starts
	glClear(GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	// Z buffer code ends */

	glClearColor(1.0, 1.0, 1.0, 0.0);	// Set display-window color to white.
	glClear(GL_COLOR_BUFFER_BIT);		// Clear display window.

	double Shadow[] = { 10, 10, 10 };

	glPushMatrix();
		double Burgundy[] = { 159, 29, 53 };
		glTranslatef(disk_pos[0].x, disk_pos[0].y, disk_pos[0].z);
		draw_disk(30, hight_disk, Burgundy);
	glPopMatrix();
	glPushMatrix();
		glTranslatef(light.x, light.y, light.z);
		glMultMatrixf(M);
		glTranslatef(-light.x, -light.y, -light.z);

		glTranslatef(disk_pos[0].x, disk_pos[0].y, disk_pos[0].z);
		draw_disk(30, hight_disk, Shadow);
	glPopMatrix();

	glPushMatrix();
		double MidnightBlue[] = { 25, 25, 112 };
		glTranslatef(disk_pos[1].x, disk_pos[1].y, disk_pos[1].z);
		draw_disk(25, hight_disk, MidnightBlue);
	glPopMatrix();
	glPushMatrix();
		glTranslatef(light.x, light.y, light.z);
		glMultMatrixf(M);
		glTranslatef(-light.x, -light.y, -light.z);

		glTranslatef(disk_pos[1].x, disk_pos[1].y, disk_pos[1].z);
		draw_disk(25, hight_disk, Shadow);
	glPopMatrix();

	glPushMatrix();
		double IrishGreen[] = { 0, 158, 96 };
		glTranslatef(disk_pos[2].x, disk_pos[2].y, disk_pos[2].z);
		draw_disk(20, hight_disk, IrishGreen);
	glPopMatrix();

	glPushMatrix();
		glTranslatef(light.x, light.y, light.z);
		glMultMatrixf(M);
		glTranslatef(-light.x, -light.y, -light.z);

		glTranslatef(disk_pos[2].x, disk_pos[2].y, disk_pos[2].z);
		draw_disk(20, hight_disk, Shadow);
	glPopMatrix();
	

	glPushMatrix();
		glTranslatef(claw_pos.x, claw_pos.y, claw_pos.z);
		draw_claw();
	glPopMatrix();

	glPushMatrix();
		glTranslatef(light.x, light.y, light.z);
		glMultMatrixf(M);
		glTranslatef(-light.x, -light.y, -light.z);

		glTranslatef(claw_pos.x, claw_pos.y, claw_pos.z);
		draw_claw(true);
	glPopMatrix();

	glutSwapBuffers();
}

void animate()
{
	if (cur_scene > 60)
		return;
	double t = (glutGet(GLUT_ELAPSED_TIME) - t_prev) / 1000;
	if (t >= sec_per_act)
	{
		t_prev = glutGet(GLUT_ELAPSED_TIME);
		cur_scene++;
		return;
	}
	cout << "cur_scene : " << cur_scene << endl;
	switch (cur_scene)
	{
		case 1:
			moveClaw('z', top_mid.z, top_left.z, t);
			break;
		case 2:
			opClaw('o', t);
			break;
		case 3:
			moveClaw('y', top_left.y, bottom_left.y + hight_total_claw + 2*hight_disk, t);
			break;
		case 4:
			opClaw('c', t);
			break;
		case 5:
			moveClaw('y', bottom_left.y + hight_total_claw + 2 * hight_disk, top_left.y, t);
			moveBaseDisk(2, 'y', bottom_left.y + 2 * hight_disk, top_left.y - hight_total_claw, t);
			break;
		case 6:
			moveClaw('z', top_left.z, top_right.z, t);
			moveBaseDisk(2, 'z', top_left.z, top_right.z, t);
			break;
		case 7:
			moveClaw('y', top_right.y, bottom_right.y + hight_total_claw, t);
			moveBaseDisk(2, 'y', top_right.y - hight_total_claw, bottom_right.y, t);
			break;
		case 8:
			opClaw('o', t);
			break;
		case 9:
			moveClaw('y', bottom_right.y + hight_total_claw, top_right.y, t);
			break;
		case 10:
			moveClaw('z', top_right.z, top_left.z, t);
			break;
		case 11:
			moveClaw('y', top_left.y, bottom_left.y + hight_total_claw + 1 * hight_disk, t);
			break;
		case 12:
			opClaw('c', t);
			break;
		case 13:
			moveClaw('y', bottom_left.y + hight_total_claw + 1 * hight_disk, top_left.y, t);
			moveBaseDisk(1, 'y', bottom_left.y + 1 * hight_disk, top_left.y - hight_total_claw, t);
			break;
		case 14:
			moveClaw('z', top_left.z, top_mid.z, t);
			moveBaseDisk(1, 'z', top_left.z, top_mid.z, t);
			break;
		case 15:
			moveClaw('y', top_mid.y, bottom_mid.y + hight_total_claw, t);
			moveBaseDisk(1, 'y', top_mid.y - hight_total_claw, bottom_mid.y, t);
			break;
		case 16:
			opClaw('o', t);
			break;
		case 17:
			moveClaw('y', bottom_mid.y + hight_total_claw, top_mid.y, t);
			break;
		case 18:
			moveClaw('z', top_mid.z, top_right.z, t);
			break;
		case 19:
			moveClaw('y', top_left.y, bottom_left.y + hight_total_claw + 0 * hight_disk, t);
			break;
		case 20:
			opClaw('c', t);
			break;
		case 21:
			moveClaw('y', bottom_left.y + hight_total_claw + 0 * hight_disk, top_left.y, t);
			moveBaseDisk(2, 'y', bottom_left.y + 0 * hight_disk, top_left.y - hight_total_claw, t);
			break;
		case 22:
			moveClaw('z', top_right.z, top_mid.z, t);
			moveBaseDisk(2, 'z', top_right.z, top_mid.z, t);
			break;
		case 23:
			moveClaw('y', top_mid.y, bottom_mid.y + hight_total_claw + 1 * hight_disk, t);
			moveBaseDisk(2, 'y', top_mid.y - hight_total_claw, bottom_mid.y + 1 * hight_disk, t);
			break;
		case 24:
			opClaw('o', t);
			break;
		case 25:
			moveClaw('y', bottom_mid.y + hight_total_claw + 1 * hight_disk, top_mid.y, t);
			break;
		case 26:
			moveClaw('z', top_mid.z, top_left.z, t);
			break;
		case 27:
			moveClaw('y', top_mid.y, bottom_mid.y + hight_total_claw + 0 * hight_disk, t);
			break;
		case 28:
			opClaw('c', t);
			break;
		case 29:
			moveClaw('y', bottom_left.y + hight_total_claw + 0 * hight_disk, top_left.y, t);
			moveBaseDisk(0, 'y', bottom_left.y + 0 * hight_disk, top_left.y - hight_total_claw, t);
			break;
		case 30:
			moveClaw('z', top_left.z, top_right.z, t);
			moveBaseDisk(0, 'z', top_left.z, top_right.z, t);
			break;
		case 31:
			moveClaw('y', top_mid.y, bottom_mid.y + hight_total_claw + 0 * hight_disk, t);
			moveBaseDisk(0, 'y', top_mid.y - hight_total_claw, bottom_mid.y + 0 * hight_disk, t);
			break;
		case 32:
			opClaw('o', t);
			break;
		case 33:
			moveClaw('y', bottom_right.y + hight_total_claw + 0 * hight_disk, top_right.y, t);
			break;
		case 34:
			moveClaw('z', top_right.z, top_mid.z, t);
			break;
		case 35:
			moveClaw('y', top_mid.y, bottom_mid.y + hight_total_claw + 1 * hight_disk, t);
			break;
		case 36:
			opClaw('c', t);
			break;
		case 37:
			moveClaw('y', bottom_mid.y + hight_total_claw + 1 * hight_disk, top_mid.y, t);
			moveBaseDisk(2, 'y', bottom_mid.y + 1 * hight_disk, top_mid.y - hight_total_claw, t);
			break;
		case 38:
			moveClaw('z', top_mid.z, top_left.z, t);
			moveBaseDisk(2, 'z', top_mid.z, top_left.z, t);
			break;
		case 39:
			moveClaw('y', top_left.y, bottom_left.y + hight_total_claw + 0 * hight_disk, t);
			moveBaseDisk(2, 'y', top_left.y - hight_total_claw, bottom_left.y + 0 * hight_disk, t);
			break;
		case 40:
			opClaw('o', t);
			break;
		case 41:
			moveClaw('y', bottom_left.y + hight_total_claw + 0 * hight_disk, top_left.y, t);
			break;
		case 42:
			moveClaw('z', top_left.z, top_mid.z, t);
			break;
		case 43:
			moveClaw('y', top_mid.y, bottom_mid.y + hight_total_claw + 0 * hight_disk, t);
			break;
		case 44:
			opClaw('c', t);
			break;
		case 45:
			moveClaw('y', bottom_mid.y + hight_total_claw + 0 * hight_disk, top_mid.y, t);
			moveBaseDisk(1, 'y', bottom_mid.y + 0 * hight_disk, top_mid.y - hight_total_claw, t);
			break;
		case 46:
			moveClaw('z', top_mid.z, top_right.z, t);
			moveBaseDisk(1, 'z', top_mid.z, top_right.z, t);
			break;
		case 47:
			moveClaw('y', top_right.y, bottom_right.y + hight_total_claw + 1 * hight_disk, t);
			moveBaseDisk(1, 'y', top_right.y - hight_total_claw, bottom_right.y + 1 * hight_disk, t);
			break;
		case 48:
			opClaw('o', t);
			break;
		case 49:
			moveClaw('y', bottom_right.y + hight_total_claw + 0 * hight_disk, top_right.y, t);
			break;
		case 50:
			moveClaw('z', top_right.z, top_left.z, t);
			break;
		case 51:
			moveClaw('y', top_left.y, bottom_left.y + hight_total_claw + 0 * hight_disk, t);
			break;
		case 52:
			opClaw('c', t);
			break;
		case 53:
			moveClaw('y', bottom_left.y + hight_total_claw + 0 * hight_disk, top_left.y, t);
			moveBaseDisk(2, 'y', bottom_left.y + 0 * hight_disk, top_left.y - hight_total_claw, t);
			break;
		case 54:
			moveClaw('z', top_left.z, top_right.z, t);
			moveBaseDisk(2, 'z', top_left.z, top_right.z, t);
			break;
		case 55:
			moveClaw('y', top_right.y, bottom_right.y + hight_total_claw + 2 * hight_disk, t);
			moveBaseDisk(2, 'y', top_right.y - hight_total_claw, bottom_right.y + 2 * hight_disk, t);
			break;
		case 56:
			opClaw('o', t);
			break;
		case 57:
			moveClaw('y', bottom_right.y + hight_total_claw + 2 * hight_disk, top_right.y, t);
			break;
		case 58:
			moveClaw('z', top_right.z, top_mid.z, t);
			break;
		default:
			break;
	}
	 glutPostRedisplay ( );
}

void init()
{
	for (int i = 0; i < 16; i++)
		M[i] = 0;
	M[0] = M[5] = M[10] = 1;
	M[7] = 1.0 / (ground + 1 - light.y);

	claw_pos.x = top_mid.x;
	claw_pos.y = top_mid.y;
	claw_pos.z = top_mid.z;

	disk_pos[0].x = org.x;
	disk_pos[0].y = org.y;
	disk_pos[0].z = org.z;

	disk_pos[1].x = org.x;
	disk_pos[1].y = org.y + hight_disk;
	disk_pos[1].z = org.z;

	disk_pos[2].x = org.x;
	disk_pos[2].y = org.y + hight_disk*2;
	disk_pos[2].z = org.z;

	theta = 0; phi = 45 * 0.3; // psi = 0;

	t_prev = glutGet(GLUT_ELAPSED_TIME);
}

int main (int argc, char** argv)
{
     
		glutInit (&argc, argv);			                      // Initialize GLUT
		glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB |  GLUT_DEPTH); // Set display mode
		
		glutInitWindowPosition( WIN_POSX, WIN_POSY );         // Set display-window position at (WIN_POSX, WIN_POSY) 
                                                              // where (0, 0) is top left corner of monitor screen
        glutInitWindowSize( WIN_WIDTH, WIN_HEIGHT );		  // Set display-window width and height.

		glutCreateWindow ("Tower of Hanoi" );					  // Create display window.

		init();
		glutIdleFunc (animate);

        glutDisplayFunc (drawscene);   // put everything you wish to draw in drawscene

		glutMainLoop ( );
	
}