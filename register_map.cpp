#include <libbldcm/register_map.hpp>

#include <libfpgasoc.hpp>
#include <memory>
#include <exception>
#include <array>
#include <string>

using std::shared_ptr;
using std::array;
using std::string;

namespace bldcm {
// Utilities
inline uint32_t pickupValue(const uint32_t value, const uint32_t bitPos, const uint32_t bitMask)
{
	return ((value & bitMask) >> bitPos);
}

inline void insertValue(uint32_t &updatedValue, const uint32_t insertedValue, const uint32_t bitPos, const uint32_t bitMask)
{
	updatedValue = (updatedValue & (~bitMask)) | ((insertedValue << bitPos) & bitMask);
}

// Register
void Register::reg(const Register &reg, const bool isOnlyWriteCache) noexcept(false)
{
	const uint32_t origCache = this->_regCache;

	this->_regCache = reg._regCache;

	if (isOnlyWriteCache) {
		this->_cacheStatus = CacheState::modified;
	} else {
		try {
			this->flushCache();
		} catch (const std::range_error &e) {
			this->_regCache = origCache;
			throw;
		}
	}
}

void Register::reg(const uint32_t &val, const bool isOnlyWriteCache) noexcept(false)
{
	const uint32_t origCache = this->_regCache;

	this->_regCache = val;

	if (isOnlyWriteCache) {
		this->_cacheStatus = CacheState::modified;
	} else {
		try {
			this->flushCache();
		} catch (const std::range_error &e) {
			this->_regCache = origCache;
			throw;
		}
	}
}

uint32_t Register::reg(const bool isReadFromCache) noexcept(false)
{
	if (!isReadFromCache) {
		this->updateCache();
	}

	return this->_regCache;
}

void Register::flushCache() noexcept(false)
{
	this->_fpgaObj.write32(this->_addr, this->_regCache);
	this->_cacheStatus = CacheState::sync;
	this->_flushCacheCallBack();
}

void Register::updateCache() noexcept(false)
{
	this->_regCache = this->_fpgaObj.read32(this->_addr);
	this->_cacheStatus = CacheState::sync;
}

Register::CacheState Register::cacheStatus() const noexcept(true)
{
	return this->_cacheStatus;
}

void Register::_forceSetCacheStatus(const Register::CacheState newState) noexcept(true)
{
	this->_cacheStatus = newState;
}

// FreqtgtReg
void FreqtgtReg::freqtgt(const uint32_t val, const bool isOnlyWriteCache) noexcept(false)
{
	this->reg(val, isOnlyWriteCache);
}

uint32_t FreqtgtReg::freqtgt(const bool isReadFromCache) noexcept(false)
{
	return this->reg(isReadFromCache);
}

// PwmCmpReg
void PwmCmpReg::pwmCmp(const uint32_t val, const bool isOnlyWriteCache) noexcept(false)
{
	uint32_t regValue = this->reg(isOnlyWriteCache);

	insertValue(regValue, val, PwmCmp::Bit::Pos, PwmCmp::Bit::Mask);

	this->reg(regValue, isOnlyWriteCache);
}

uint32_t PwmCmpReg::pwmCmp(const bool isReadFromCache) noexcept(false)
{
	return pickupValue(this->reg(isReadFromCache), PwmCmp::Bit::Pos, PwmCmp::Bit::Mask);
}

// CtrlReg
void CtrlReg::pwmMaxcnt(const uint16_t val, const bool isOnlyWriteCache) noexcept(false)
{
	uint32_t regValue = this->reg(isOnlyWriteCache);

	insertValue(regValue, static_cast<uint32_t>(val), PwmMaxcnt::Bit::Pos, PwmMaxcnt::Bit::Mask);

	this->reg(regValue, isOnlyWriteCache);
}

uint16_t CtrlReg::pwmMaxcnt(const bool isReadFromCache) noexcept(false)
{
	const uint32_t ret = pickupValue(this->reg(isReadFromCache), PwmMaxcnt::Bit::Pos, PwmMaxcnt::Bit::Mask);
	return static_cast<uint16_t>(ret);
}

void CtrlReg::pwmPrsc(const uint8_t val, const bool isOnlyWriteCache) noexcept(false)
{
	uint32_t regValue = this->reg(isOnlyWriteCache);

	insertValue(regValue, static_cast<uint32_t>(val), PwmPrsc::Bit::Pos, PwmPrsc::Bit::Mask);

	this->reg(regValue, isOnlyWriteCache);
}

uint8_t CtrlReg::pwmPrsc(const bool isReadFromCache) noexcept(false)
{
	const uint32_t ret = pickupValue(this->reg(isReadFromCache), PwmPrsc::Bit::Pos, PwmPrsc::Bit::Mask);
	return static_cast<uint8_t>(ret);
}

void CtrlReg::phase(const uint8_t val, const bool isOnlyWriteCache) noexcept(false)
{
	uint32_t regValue = this->reg(isOnlyWriteCache);

	// Insert PHASE
	insertValue(regValue, static_cast<uint32_t>(val), Phase::Bit::Pos, Phase::Bit::Mask);
	// Insert W_PHASE
	insertValue(regValue, static_cast<uint32_t>(WPhase::Val::Write), WPhase::Bit::Pos, WPhase::Bit::Mask);

	this->reg(regValue, isOnlyWriteCache);
}

uint8_t CtrlReg::phase(const bool isReadFromCache) noexcept(false)
{
	const uint32_t ret = pickupValue(this->reg(isReadFromCache), Phase::Bit::Pos, Phase::Bit::Mask);
	return static_cast<uint8_t>(ret);
}

void CtrlReg::_flushCacheCallBack() noexcept(true)
{
	// After writing back to register, W_PHASE bit of cache must be clear.
	uint32_t regValue = this->reg(true);
	insertValue(regValue, static_cast<uint32_t>(WPhase::Val::NotWrite), WPhase::Bit::Pos, WPhase::Bit::Mask);
	this->reg(regValue, true);
	this->_forceSetCacheStatus(CacheState::sync);
}

void CtrlReg::en(const uint8_t val, const bool isOnlyWriteCache) noexcept(false)
{
	uint32_t regValue = this->reg(isOnlyWriteCache);

	insertValue(regValue, static_cast<uint32_t>(val), En::Bit::Pos, En::Bit::Mask);

	this->reg(regValue, isOnlyWriteCache);
}

uint8_t CtrlReg::en(const bool isReadFromCache) noexcept(false)
{
	const uint32_t ret = pickupValue(this->reg(isReadFromCache), En::Bit::Pos, En::Bit::Mask);
	return static_cast<uint8_t>(ret);
}

// StatReg
const array<string, StatReg::RelCnt::MaxVal+1> StatReg::RelCnt::VerTbl = {
	"UNDR 2.10",
	"2.10"
};

uint8_t StatReg::relCnt(const bool isReadFromCache) noexcept(false)
{
	const uint32_t ret = pickupValue(this->reg(isReadFromCache), RelCnt::Bit::Pos, RelCnt::Bit::Mask);
	return static_cast<uint8_t>(ret);
}

uint8_t StatReg::deadtime(const bool isReadFromCache) noexcept(false)
{
	const uint32_t ret = pickupValue(this->reg(isReadFromCache), Deadtime::Bit::Pos, Deadtime::Bit::Mask);
	return static_cast<uint8_t>(ret);
}

uint8_t StatReg::reflectedfreq(const bool isReadFromCache) noexcept(false)
{
	const uint32_t ret = pickupValue(this->reg(isReadFromCache), Reflectedfreq::Bit::Pos, Reflectedfreq::Bit::Mask);
	return static_cast<uint8_t>(ret);
}

uint8_t StatReg::stop(const bool isReadFromCache) noexcept(false)
{
	const uint32_t ret = pickupValue(this->reg(isReadFromCache), Stop::Bit::Pos, Stop::Bit::Mask);
	return static_cast<uint8_t>(ret);
}

} // End of "namespace bldcm"

