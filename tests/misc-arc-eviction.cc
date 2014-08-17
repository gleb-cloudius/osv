#include <iostream>
#include <iomanip>
#include <string>
#include <map>
#include <random>
#include <atomic>
#include <thread>
#include <boost/iterator/function_input_iterator.hpp>

#include <sys/mman.h>
#include <sys/sysinfo.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

constexpr unsigned PAGE_SIZE  = 4096;

class rnd_seq {
    std::default_random_engine _rd;
    std::mt19937 _gen;
    std::normal_distribution<> _d;
    unsigned long _size;
public:
    typedef int result_type;
    rnd_seq(unsigned long size) : _gen(_rd()), _d(0.5, 0.2), _size(size-1) {};
    int operator()() {
        double x = -1;
        while (x < 0 || x > 1) 
            x = _d(_gen);
        return std::round(x*_size);
    }
};

int main(int argc, char **argv)
{
    struct sysinfo sinfo;
    assert(sysinfo(&sinfo) == 0);
    std::cout << "Total Ram " << (sinfo.totalram >> 20) << " Mb\n";
    unsigned long pages = sinfo.totalram / PAGE_SIZE;
    unsigned long mapsize = pages * 0.3;//663;
    rnd_seq seq(mapsize);
    std::vector<unsigned> incore(mapsize);
    std::vector<unsigned> hit(mapsize);

    int fd = open("/tmp/tst-arc", O_CREAT | O_RDWR, 0777);
    //  ftruncate(fd, mapsize*PAGE_SIZE);
    for (unsigned i = 0; i < mapsize; i++) {
        char page[PAGE_SIZE];
        write(fd, page, PAGE_SIZE);
    }
    printf("mapsize=%u\n", mapsize);
    volatile char *mem = (char*)mmap(NULL, mapsize * PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

    std::atomic<unsigned long> cache(0), total(0);

    auto func = [&](bool p) {
        std::for_each(boost::make_function_input_iterator(seq, 0ul),
                boost::make_function_input_iterator(seq, mapsize * 1000),
                [&](int i) {
            unsigned char c = 0;
//            mincore((void*)&mem[i*PAGE_SIZE], PAGE_SIZE, &c);
            total++;
            auto a __attribute__ ((unused)) = mem[i*PAGE_SIZE];
            hit[i]++;
            if (c) {
                incore[i]++;
                cache++;
            }
            if (p && !(total % 10000))
                fprintf(stderr, "\rtotal=%llu hits=%llu", total.load(std::memory_order_relaxed), cache.load(std::memory_order_relaxed)*100/total.load(std::memory_order_relaxed));
        } );
    };

    std::thread t1(func, true), t2(func, false), t3(func, false), t4(func, false);

    t1.join();
    t2.join();
    t3.join();
    t4.join();

    for (unsigned i = 0; i < mapsize; i++)
        std::cout << i << ' ' << hit[i] << ' ' << incore[i] << '\n';

    return 0;
}
