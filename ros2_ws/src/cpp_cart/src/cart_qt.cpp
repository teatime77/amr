// #include <QGuiApplication>
// #include <QMainWindow>
#include <QApplication>
#include <QGridLayout>
#include <QSlider>
#include <QLabel>
#include <QPushButton>

static QApplication* app;
static QWidget*      win;
static QSlider*      pwmL;
static QSlider*      pwmR;

void init_qt(int argc, char * argv[]){
    app = new QApplication(argc, argv);
    win = new QWidget();
    win->setWindowTitle("grid_layout");

    QGridLayout* layout = new QGridLayout;
    pwmL = new QSlider(Qt::Horizontal);
    pwmL->setRange(-255, 255);
    pwmL->setValue(0);

    pwmR = new QSlider(Qt::Horizontal);
    pwmR->setRange(-255, 255);
    pwmR->setValue(0);


    layout->addWidget(new QLabel("PWM Left"), 0, 0, Qt::AlignLeft);
    layout->addWidget(pwmL, 0, 1, Qt::AlignLeft);

    layout->addWidget(new QLabel("PWM Right"), 1, 0, Qt::AlignLeft);
    layout->addWidget(pwmR, 1, 1, Qt::AlignLeft);

    //グリッド(表)の周りの隙間の大きさを設定
    layout->setContentsMargins(20, 10, 20, 10);

    //GUI部品間の間隔を設定
    layout->setSpacing(5);    
        

    win->setLayout(layout);
    win->show();
}

void process_qt(short& pwm_l, short& pwm_r){
    app->processEvents();

    pwm_l = pwmL->value();
    pwm_r = pwmR->value();
}
