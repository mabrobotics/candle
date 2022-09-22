
#include <condition_variable>
#include <mutex>

class Semaphore
{
   public:
	Semaphore(int count_ = 0)
		: count(count_) {}

	inline void notify()
	{
		std::unique_lock<std::mutex> lock(mtx);
		count++;
		cv.notify_one();
		std::cout << "SEM: " << count << std::endl;
	}

	inline void wait()
	{
		std::unique_lock<std::mutex> lock(mtx);

		while (count == 0)
		{
			cv.wait(lock);
		}
		count--;
	}

	inline void reset()
	{
		std::unique_lock<std::mutex> lock(mtx);
		count = 0;
	}

   private:
	std::mutex mtx;
	std::condition_variable cv;
	int count;
};