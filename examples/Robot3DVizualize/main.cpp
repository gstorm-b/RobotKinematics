#include "mainwindow.h"

#include <QApplication>
#include <QCommandLineParser>
#include <QFile>

#include <vtkAutoInit.h>

VTK_MODULE_INIT(vtkInteractionStyle)
VTK_MODULE_INIT(vtkRenderingOpenGL2)

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QFile styleSheet(QStringLiteral(":/styles/robot3dvisualize.qss"));
    if (styleSheet.open(QIODevice::ReadOnly | QIODevice::Text)) {
        a.setStyleSheet(QString::fromUtf8(styleSheet.readAll()));
    }

    QCommandLineParser parser;
    parser.addHelpOption();
    const QCommandLineOption robotOption(QStringLiteral("robot"),
                                         QStringLiteral("Robot model to open: MZ04D (default) or MZ07F."),
                                         QStringLiteral("model"),
                                         QStringLiteral("MZ04D"));
    parser.addOption(robotOption);
    parser.process(a);

    // Heap-allocated so the robot-model selector can replace this window with a new one.
    auto* w = new MainWindow(parser.value(robotOption));
    w->setAttribute(Qt::WA_DeleteOnClose);
    w->show();
    return QApplication::exec();
}
