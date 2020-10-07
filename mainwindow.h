#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QActionGroup>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    // Members
    QActionGroup* mode_group;

private slots:


    // Load / save functionality
    void on_actionLoad_triggered();
    void on_actionSave_triggered();


    // Set the UI mode
    void on_actionSelect_and_Edit_Links_triggered();

    void on_actionSelect_and_Edit_Joints_triggered();

    // reset view
    void on_actionReset_View_triggered();


    void on_actionSelect_and_Edit_Obstacles_triggered();

    void on_actionShowtime_triggered();

    void on_actionTest_Eigen_triggered();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
