#ifndef REGISTER_MAP_HPP
#define REGISTER_MAP_HPP

#include <libfpgasoc.hpp>

#include <cstdint>
#include <memory>
#include <array>
#include <string>

namespace bldcm {
class Register {
	public:
		// Type define
		enum class CacheState {
			initialized, // Never read/write register
			sync,        // Readed/writed register but unmodified.
			modified     // Modified cache since the last read from register.
		};

		// Constructor/Destructor
		virtual ~Register() {}

		// Methods
		void reg(const Register &reg, const bool isOnlyWriteCache = false) noexcept(false);
		void reg(const uint32_t &val, const bool isOnlyWriteCache = false) noexcept(false);
		uint32_t reg(const bool isReadFromCache = false) noexcept(false);

		void flushCache() noexcept(false);
		void updateCache() noexcept(false);

		CacheState cacheStatus() const noexcept(true);

	protected:
		// Only subclass can use this.
		Register(const uint32_t addr, const uint32_t resetVal, Fpgasoc &obj)
			: _addr(addr), _regCache(resetVal), _cacheStatus(CacheState::initialized), _fpgaObj(obj) {}

		void _forceSetCacheStatus(const CacheState newState) noexcept(true);

		// Subclass can override.
		virtual void _flushCacheCallBack() noexcept(true) {}

	private:
		const uint32_t _addr; // Address based on FPGA LW
		uint32_t _regCache; // Register cache
		CacheState _cacheStatus;
		Fpgasoc &_fpgaObj;

};

class FreqtgtReg : public Register {
	public:
		// Constructor/Destructor
		FreqtgtReg(Fpgasoc &obj, const uint32_t baseAddr)
			: Register(baseAddr + _Offset, _ResetVal, obj) {}
		~FreqtgtReg() override {}

		// Methods
		void freqtgt(const uint32_t val, const bool isOnlyWriteCache = false) noexcept(false);
		uint32_t freqtgt(const bool isReadFromCache = false) noexcept(false);

		// Materials
		struct Freqtgt {
			struct Bit {
				static constexpr uint32_t Mask  = static_cast<uint32_t>(0xFFFFFFFFU);
				static constexpr uint32_t Pos   = static_cast<uint32_t>(0U);
				static constexpr uint32_t Width = static_cast<uint32_t>(32U);
			};
		};

	private:
		static constexpr uint32_t _Offset   = static_cast<uint32_t>(0x00000000U);
		static constexpr uint32_t _ResetVal = static_cast<uint32_t>(0x00000000U);
};

class PwmCmpReg : public Register {
	public:
		// Constructor/Destructor
		PwmCmpReg(Fpgasoc &obj, const uint32_t baseAddr)
			: Register(baseAddr + _Offset, _ResetVal, obj) {}
		~PwmCmpReg() override {}

		// Methods
		void pwmCmp(const uint32_t val, const bool isOnlyWriteCache = false) noexcept(false);
		uint32_t pwmCmp(const bool isReadFromCache = false) noexcept(false);

		// Materials
		struct PwmCmp {
			struct Bit {
				static constexpr uint32_t Mask  = static_cast<uint32_t>(0x0001FFFFU);
				static constexpr uint32_t Pos   = static_cast<uint32_t>(0U);
				static constexpr uint32_t Width = static_cast<uint32_t>(17U);
			};
		};

	private:
		static constexpr uint32_t _Offset   = static_cast<uint32_t>(0x00000004U);
		static constexpr uint32_t _ResetVal = static_cast<uint32_t>(0x00000000U);
};

class CtrlReg : public Register {
	public:
		// Constructor/Destructor
		CtrlReg(Fpgasoc &obj, const uint32_t baseAddr)
			: Register(baseAddr + _Offset, _ResetVal, obj) {}
		~CtrlReg() override {}

		// Methods
		void pwmMaxcnt(const uint16_t val, const bool isOnlyWriteCache = false) noexcept(false);
		uint16_t pwmMaxcnt(const bool isReadFromCache = false) noexcept(false);

		void pwmPrsc(const uint8_t val, const bool isOnlyWriteCache = false) noexcept(false);
		uint8_t pwmPrsc(const bool isReadFromCache = false) noexcept(false);

		void phase(const uint8_t val, const bool isOnlyWriteCache = false) noexcept(false);
		uint8_t phase(const bool isReadFromCache = false) noexcept(false);

		void en(const uint8_t val, const bool isOnlyWriteCache = false) noexcept(false);
		uint8_t en(const bool isReadFromCache = false) noexcept(false);

		// Materials
		struct PwmMaxcnt {
			struct Bit {
				static constexpr uint32_t Mask  = static_cast<uint32_t>(0x0FFFF000U);
				static constexpr uint32_t Pos   = static_cast<uint32_t>(12U);
				static constexpr uint32_t Width = static_cast<uint32_t>(16U);
			};
		};

		struct PwmPrsc {
			struct Bit {
				static constexpr uint32_t Mask  = static_cast<uint32_t>(0x00000FC0U);
				static constexpr uint32_t Pos   = static_cast<uint32_t>(6U);
				static constexpr uint32_t Width = static_cast<uint32_t>(6U);
			};
		};

		struct WPhase {
			struct Bit {
				static constexpr uint32_t Mask  = static_cast<uint32_t>(0x00000020U);
				static constexpr uint32_t Pos   = static_cast<uint32_t>(5U);
				static constexpr uint32_t Width = static_cast<uint32_t>(1U);
			};
			struct Val {
				static constexpr uint8_t NotWrite = static_cast<uint8_t>(0x00U);
				static constexpr uint8_t Write    = static_cast<uint8_t>(0x01U);
			};
		};

		struct Phase {
			struct Bit {
				static constexpr uint32_t Mask  = static_cast<uint32_t>(0x0000001CU);
				static constexpr uint32_t Pos   = static_cast<uint32_t>(2U);
				static constexpr uint32_t Width = static_cast<uint32_t>(3U);
			};
		};

		struct En {
			struct Bit {
				static constexpr uint32_t Mask  = static_cast<uint32_t>(0x00000001U);
				static constexpr uint32_t Pos   = static_cast<uint32_t>(0U);
				static constexpr uint32_t Width = static_cast<uint32_t>(1U);
			};
			struct Val {
				static constexpr uint8_t Disable = static_cast<uint8_t>(0x00U);
				static constexpr uint8_t Enable  = static_cast<uint8_t>(0x01U);
			};
		};

	private:
		static constexpr uint32_t _Offset   = static_cast<uint32_t>(0x00000008U);
		static constexpr uint32_t _ResetVal = static_cast<uint32_t>(0x0FFFF000U);

		void _flushCacheCallBack() noexcept(true) override;
};

class StatReg : public Register {
	public:
		// Constructor/Destructor
		StatReg(Fpgasoc &obj, const uint32_t baseAddr)
			: Register(baseAddr + _Offset, _ResetVal, obj) {}
		~StatReg() override {}

		// Methods
		uint8_t relCnt(const bool isReadFromCache = false) noexcept(false);
		uint8_t deadtime(const bool isReadFromCache = false) noexcept(false);
		uint8_t reflectedfreq(const bool isReadFromCache = false) noexcept(false);
		uint8_t stop(const bool isReadFromCache = false) noexcept(false);

		// Materials
		struct RelCnt {
			struct Bit {
				static constexpr uint32_t Mask  = static_cast<uint32_t>(0xFF000000U);
				static constexpr uint32_t Pos   = static_cast<uint32_t>(24U);
				static constexpr uint32_t Width = static_cast<uint32_t>(8U);
			};
			static constexpr uint8_t MaxVal = 1;
			static const std::array<std::string, MaxVal+1> VerTbl;
		};

		struct Deadtime {
			struct Bit {
				static constexpr uint32_t Mask  = static_cast<uint32_t>(0x00F00000U);
				static constexpr uint32_t Pos   = static_cast<uint32_t>(20U);
				static constexpr uint32_t Width = static_cast<uint32_t>(4U);
			};
		};

		struct Reflectedfreq {
			struct Bit {
				static constexpr uint32_t Mask  = static_cast<uint32_t>(0x00000002U);
				static constexpr uint32_t Pos   = static_cast<uint32_t>(1U);
				static constexpr uint32_t Width = static_cast<uint32_t>(1U);
			};
			struct Val {
				static constexpr uint8_t NotReflected = static_cast<uint8_t>(0x00);
				static constexpr uint8_t Reflected    = static_cast<uint8_t>(0x01);
			};
		};

		struct Stop {
			struct Bit {
				static constexpr uint32_t Mask  = static_cast<uint32_t>(0x00000001U);
				static constexpr uint32_t Pos   = static_cast<uint32_t>(0U);
				static constexpr uint32_t Width = static_cast<uint32_t>(1U);
			};
			struct Val {
				static constexpr uint8_t Rotating = static_cast<uint8_t>(0x00);
				static constexpr uint8_t Stopping = static_cast<uint8_t>(0x01);
			};
		};

		// Invalidated these functions because they are not used.
		using Register::reg;
		void reg(const Register &reg, const bool isOnlyWriteCache) noexcept(false) = delete;
		void reg(const uint32_t &val, const bool isOnlyWriteCache) noexcept(false) = delete;
		void flushCache() noexcept(false) = delete;

	private:
		static constexpr uint32_t _Offset   = static_cast<uint32_t>(0x0000000CU);
		static constexpr uint32_t _ResetVal = static_cast<uint32_t>(0x00000000U);
};

class RegMap {
	public: 
		// Constructor/Destructor
		RegMap(const std::shared_ptr<Fpgasoc> &ptr, const uint32_t baseAddr)
			: _fpgaObjPtr(ptr),
		          freqtgt(*_fpgaObjPtr, baseAddr), pwmCmp(*_fpgaObjPtr, baseAddr),
		          ctrl(*_fpgaObjPtr, baseAddr), stat(*_fpgaObjPtr, baseAddr) {}
		~RegMap() {}

	private:
		std::shared_ptr<Fpgasoc> _fpgaObjPtr;

	public:
		// Registers
		FreqtgtReg freqtgt;
		PwmCmpReg  pwmCmp;
		CtrlReg    ctrl;
		StatReg    stat;

};
} // End of "namespace bldcm"

#endif // End of "#ifndef REGISTER_MAP_HPP"

