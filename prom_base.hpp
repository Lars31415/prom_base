#ifndef PROM_BASE_HPP
#define PROM_BASE_HPP

#include "i2c_scanner.hpp"
#include <cstring>

/**
 * @brief Base class for EEPROM and FRAM devices using I2C.
 */
class PromBase
{
public:
    /**
     * @brief Constructor for PromBase class.
     * @param i2c The I2C interface to use.
     * @param addr The address of the device on the I2C bus.
     */
    PromBase(i2c_inst_t *i2c, const uint8_t &addr);

    /**
     * @brief Destructor for PromBase class.
     * @throws None
     */
    virtual ~PromBase() throw();

    /**
     * @brief Write data to the chip.
     *
     * @param addr The address to write to.
     * @param bytes The data to write.
     * @param len The length of the data to write.
     * @return int The number of bytes written, or a negative error code.
     */
    virtual int write(uint16_t addr, const void *bytes, size_t len) const = 0;

    /**
     * @brief Read data from the chip.
     *
     * @param addr The address to read from.
     * @param bytes The buffer to read into.
     * @param len The length of the data to read.
     * @return int The number of bytes read, or a negative error code.
     */
    virtual int read(const uint16_t &addr, const void *bytes, size_t len) const = 0;

    /**
     * @brief Check if the chip is present on the bus.
     *
     * @return true The chip is present.
     * @return false The chip is not present.
     */
    bool is_present() const;

    /**
     * @brief Get the size of the EEPROM in bytes.
     *
     * @return size_t Size of the EEPROM in bytes.
     */
    virtual size_t size() const = 0; // bytes

    /**
     * @brief Check if the size of the device is plausible.
     *
     * @return true The size is plausible.
     * @return false The size is not plausible.
     */
    bool size_plausible() const;

    /**
     * @brief Perform a size test on the device.
     *
     * @return The size of the device if successful, 0 otherwise.
     */
    size_t test_size() const;

protected:
    i2c_inst_t *i2c_;     /**< The I2C instance to use. */
    const uint8_t addr_; /**< The I2C address of the chip. */

private:
    // Disallow copy and assignment
    PromBase(const PromBase &);
    PromBase &operator=(const PromBase &);
};

#endif // PROM_BASE_HPP
