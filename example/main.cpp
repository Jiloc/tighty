#include <tighty/camera/realsense/RSCamera.h>

#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QObject>

#include "videoplayer.h"


int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);
    QQmlApplicationEngine engine;

    VideoPlayer::declareQML();

    RSCamera scanner(QString("610205001283"));

    engine.rootContext()->setContextProperty("scanner", &scanner);

    engine.load(QUrl(QStringLiteral("qrc:/main.qml")));
    if (engine.rootObjects().isEmpty())
        return -1;

    return app.exec();
}
