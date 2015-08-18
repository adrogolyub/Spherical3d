#ifndef UTILITYIMAGE
#define UTILITYIMAGE

struct ThresholdOperation : std::unary_function<unsigned char, void>
{
    ThresholdOperation(unsigned char threshold) : m_threshold(threshold) {}
    void operator() (unsigned char& value)
    {
        value = value > m_threshold ? 1 : 0;
    }
private:
    unsigned char m_threshold;
};

struct UtilityImage
{
    unsigned char* data;
    int width;
    int height;

    UtilityImage() : data(NULL), width(0), height(0) {}

    void Free()
    {
        kutility::deallocate(data);
    }

    void ApplyThreshold(unsigned char threshold)
    {
        if (data == NULL)
        {
            return;
        }

        std::for_each(data, data + width * height, ThresholdOperation(threshold));
    }

    ~UtilityImage()
    {
        Free();
    }
};

#endif // UTILITYIMAGE

