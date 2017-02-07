#ifndef WINDOW_HPP_
#define WINDOW_HPP_

#include "FAST/Object.hpp"
#include "WindowWidget.hpp"
#include "ComputationThread.hpp"
#include "View.hpp"
#include <vector>
#include <QOpenGLContext>

class QOpenGLContext;
class QEventLoop;
class QApplication;
class QOffscreenSurface;

namespace fast {

class Window : public QObject, public Object {
    Q_OBJECT
    public:
        static void initializeQtApp();
        static QOpenGLContext* getMainGLContext();
        static QSurface* getSurface();
        // Makes the window close after a specific number of ms
        virtual void setTimeout(unsigned int milliseconds);
        ~Window();
        virtual void start();
        void startComputationThread();
        void stopComputationThread();
        void setWidth(uint width);
        void setHeight(uint height);
        void enableFullscreen();
        void disableFullscreen();
        std::vector<View*> getViews() const;
        static void cleanup();
    protected:
        Window();
        View* createView();
        View* getView(uint i) const;
        static QOpenGLContext* mMainGLContext;
        static QOffscreenSurface* mSurface;

        WindowWidget* mWidget;
        unsigned int mWidth, mHeight;
        bool mFullscreen;
        unsigned int mTimeout;
        QEventLoop* mEventLoop;
        ComputationThread* mThread;
        static QApplication* mQApp;
    public slots:
        void stop();


};

} // end namespace fast

#endif
