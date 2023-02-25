// #include <QGuiApplication>
// #include <QMainWindow>
#include <QApplication>
#include <QFormLayout>
#include <QGridLayout>
#include <QSlider>
#include <QLabel>
#include <QPushButton>

static QApplication* app;
static QWidget*      win;
static QSlider*      pwmVel;
static QSlider*      pwmDir;

void init_qt(int argc, char * argv[]){
    app = new QApplication(argc, argv);
    win = new QWidget();
    win->setWindowTitle("PWM Velocity & Direction");
    win->setFixedWidth(800);

    QFormLayout* layout = new QFormLayout;  // QGridLayout;
    pwmVel = new QSlider(Qt::Horizontal);
    pwmVel->setRange(-255, 255);
    pwmVel->setValue(0);
    pwmVel->resize(300, 20);

    pwmDir = new QSlider(Qt::Horizontal);
    pwmDir->setRange(-255, 255);
    pwmDir->setValue(0);
    pwmDir->resize(300, 20);


    layout->addRow(new QLabel("PWM Vel"), pwmVel);
    layout->addRow(new QLabel("PWM Dir"), pwmDir);

    win->setLayout(layout);
    win->show();
}

void process_qt(int& pwm_l, int& pwm_r){
    app->processEvents();

    pwm_l =  pwmVel->value();
    pwm_r = -pwmVel->value();

    pwm_l += pwmDir->value();
    pwm_r += pwmDir->value();

    pwm_l = std::max(-255, std::min(255, pwm_l));
    pwm_r = std::max(-255, std::min(255, pwm_r));
}
