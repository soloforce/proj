#include <pthread.h>

class Thread
{
public:
    virtual ~Thread() {}
    pthread_t start()
    {
        pthread_create(&_tid, NULL, hook, this);
        return tid;
    }

private:
    void* hook(void* args)
    {
        reinterpret_cast<Thread*>(args)->worker();
        return NULL;
    }

protected:
    pthread_t _tid;
protected:
    virtual void worker() = 0;
};

class GCThread : public Thread
{
public
protected:
    int fd;
protected:
    virtual void worker()
    {


    }
};
