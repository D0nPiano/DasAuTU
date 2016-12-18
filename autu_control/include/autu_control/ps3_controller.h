#ifndef PS3_CONTROLLER_H
#define PS3_CONTROLLER_H

#include <QObject>

class PS3_Controller : public QObject
{
    Q_OBJECT
public:
    explicit PS3_Controller(QObject *parent = 0);

signals:

public slots:
};

#endif // PS3_CONTROLLER_H