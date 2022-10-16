#include <DialogKeyboard.h>
#include <QtWidgets>
#include<unistd.h>
#include "ui_DialogKeyboard.h"

int wp=0;
double vetorwpx[6]={5,2,4,7,2,0};
double vetorwpy[6]={4,1,6,2,3,0};
unsigned int microsecond = 1000000;

DialogKeyboard::DialogKeyboard(QWidget *parent)
    : QDialog(parent), ui(new Ui::DialogKeyboard) {
  ui->setupUi(this);
}

DialogKeyboard::~DialogKeyboard() { delete ui; }

void DialogKeyboard::keyPressEvent(QKeyEvent *event) {
  if (!drone)
    return;
  char key = event->key();
  std::cout << "key:" << key << std::endl;
  switch (key) {
  case 'Z':
    // take off
    std::cout << "take off !" << std::endl;
    drone->takeOff();
    break;
  case 'X':
    // land
    drone->land();
    break;
  case 'H':
    drone->hover();
    break;
  case 'I':
    // going up
    drone->rise(0.4f);
    break;
  case 'K':
    // going down
    drone->rise(-0.4f);
    break;
  case 'J':
    // turn left
    drone->yaw(0.4f);
    break;
  case 'L':
    // turn right
    drone->yaw(-0.4f);
    break;
  case 'A':
    // tilt left
    drone->roll(0.7f);
    break;
  case 'D':
    // tilt left
    drone->roll(-0.7f);
    break;
  case 'W':
    // title front
    drone->pitch(0.7f);
    break;
  case 'S':
    // title back
    drone->pitch(-0.7f);
    break;
  case 'T':
    testPositionControl();
    break;
  case 'P':
    waypointPose();
    break;

  default:
    drone->hover();
  }
  event->accept();
}

void DialogKeyboard::keyReleaseEvent(QKeyEvent *event) {
  if (!drone)
    return;
  char key = event->key();
  if (!event->isAutoRepeat()) {
    std::cout << "key:" << key << " has been released !" << std::endl;
    if (!drone->isPosctrl)
      drone->hover();
    event->accept();
  } else {
    event->ignore();
  }
}

void DialogKeyboard::testPositionControl() {
  if (drone->isPosctrl) {
    drone->posCtrl(false);
    std::cout << "position control off!" << std::endl;
  } else {
    drone->posCtrl(true);
    std::cout << "Flying to (4.5,-4.5, 6) with position control." << std::endl;
    drone->moveTo(4.5, -4.5, 6);
  }
}

void DialogKeyboard::waypointPose() {
  if (drone->isPosctrl) {
    drone->posCtrl(false);
    std::cout << "position control off!" << std::endl;
  } else {
    drone->posCtrl(true);
    while (wp < 6) {
      ROS_INFO("Flying to (%f, %f, 5) with position control",vetorwpx[wp], vetorwpy[wp]);
      drone->moveTo(vetorwpx[wp], vetorwpy[wp], 5);
      wp++;
      usleep(10 * microsecond);//sleeps for 3 second
    }
    
  }
}