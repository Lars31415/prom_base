#include "prom_base.hpp"
#include "pico_errors.hpp"

#include <iostream>
/**
 * @brief Constructor for PromBase class.
 * @param i2c The I2C interface to use.
 * @param addr The address of the device on the I2C bus.
 */
PromBase::PromBase(i2c_inst_t *i2c, const uint8_t &addr)
    : i2c_(i2c), addr_(addr)
{
}

/**
 * @brief Destructor for PromBase class.
 * @throws None
 */
PromBase::~PromBase() throw()
{
}

/**
 * @brief Check if the chip is present on the bus.
 *
 * @return true The chip is present.
 * @return false The chip is not present.
 */
bool PromBase::is_present() const
{
    return i2c_device_present(i2c_, addr_);
}

/**
 * @brief Check if the size of the device is plausible.
 *
 * @return true The size is plausible.
 * @return false The size is not plausible.
 */
bool PromBase::size_plausible() const
{
    return (test_size() == size());
}

/**
 * @brief Perform a size test on the device.
 *
 * @return The size of the device if successful, 0 otherwise.
 */
size_t PromBase::test_size() const
{
    bool flag = false;
    size_t j = 0x0000;
    uint8_t buf1[4];
    uint8_t buf2[4];
    int err = 0;

    // Perform a read at address 0 and check for errors
    if ((err = read(0, buf1, 4)) < 0)
        return err;

    do
    {
        j += 0x1000;

        // Perform a read at address j and check for errors
        if ((err = read(j, buf2, 4)) < 0)
        {
            return err;
        }

        // Check if the read data matches the initial data
        flag = (memcmp(buf1, buf2, 4) == 0);

        // If the data matches, perform another read at 2*j and check for errors
        if (flag)
        {
            if ((err = read(2 * j, buf2, 4)) < 0)
            {
                return err;
            }
            // Check if the read data at 2*j matches the initial data
            flag = (memcmp(buf1, buf2, 4) == 0);
        }
    } while ((j < 0x8000) && (!flag));

    // Return the size of the device
    return j;
}
