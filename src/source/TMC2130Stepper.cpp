#include "TMCStepper.h"
#include "TMC_MACROS.h"
#include <cstring>

#ifdef USE_ZERODMA
namespace {
enum AsyncPhase : uint8_t {
  PHASE_READ_CMD = 0,
  PHASE_SHIFT_PRE = 1,
  PHASE_SHIFT_POST = 2,
  PHASE_READ_DATA = 3,
  PHASE_WRITE_CMD = 4,
  PHASE_WRITE_SHIFT = 5
};

inline void fillZeroFrame(uint8_t* buf) {
  buf[0] = 0x00;
  buf[1] = 0x00;
  buf[2] = 0x00;
  buf[3] = 0x00;
  buf[4] = 0x00;
}
}
#endif // USE_ZERODMA

int8_t TMC2130Stepper::chain_length = 0;
uint32_t TMC2130Stepper::spi_speed = 16000000/8;

TMC2130Stepper::TMC2130Stepper(uint16_t pinCS, float RS, int8_t link) :
  TMCStepper(RS),
  _pinCS(pinCS),
  link_index(link)
  {
    defaults();

    if (link > chain_length)
      chain_length = link;
  }

TMC2130Stepper::TMC2130Stepper(uint16_t pinCS, uint16_t pinMOSI, uint16_t pinMISO, uint16_t pinSCK, int8_t link) :
  TMCStepper(default_RS),
  _pinCS(pinCS),
  link_index(link)
  {
    SW_SPIClass *SW_SPI_Obj = new SW_SPIClass(pinMOSI, pinMISO, pinSCK);
    TMC_SW_SPI = SW_SPI_Obj;
    defaults();

    if (link > chain_length)
      chain_length = link;
  }

TMC2130Stepper::TMC2130Stepper(uint16_t pinCS, float RS, uint16_t pinMOSI, uint16_t pinMISO, uint16_t pinSCK, int8_t link) :
  TMCStepper(RS),
  _pinCS(pinCS),
  link_index(link)
  {
    SW_SPIClass *SW_SPI_Obj = new SW_SPIClass(pinMOSI, pinMISO, pinSCK);
    TMC_SW_SPI = SW_SPI_Obj;
    defaults();

    if (link > chain_length)
      chain_length = link;
  }

void TMC2130Stepper::defaults() {
  //MSLUT0_register.sr = ???;
  //MSLUT1_register.sr = ???;
  //MSLUT2_register.sr = ???;
  //MSLUT3_register.sr = ???;
  //MSLUT4_register.sr = ???;
  //MSLUT5_register.sr = ???;
  //MSLUT6_register.sr = ???;
  //MSLUT7_register.sr = ???;
  //MSLUTSTART_register.start_sin90 = 247;
  PWMCONF_register.sr = 0x00050480;
}

__attribute__((weak))
void TMC2130Stepper::setSPISpeed(uint32_t speed) {
  spi_speed = speed;
}

__attribute__((weak))
void TMC2130Stepper::switchCSpin(bool state) {
  digitalWrite(_pinCS, state);
}

__attribute__((weak))
void TMC2130Stepper::beginTransaction() {
  if (TMC_SW_SPI == nullptr) {
    SPI.beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));
  }
}
__attribute__((weak))
void TMC2130Stepper::endTransaction() {
  if (TMC_SW_SPI == nullptr) {
    SPI.endTransaction();
  }
}

__attribute__((weak))
uint8_t TMC2130Stepper::transfer(const uint8_t data) {
  uint8_t out = 0;
  if (TMC_SW_SPI != nullptr) {
    out = TMC_SW_SPI->transfer(data);
  }
  else {
    out = SPI.transfer(data);
  }
  return out;
}

void TMC2130Stepper::transferEmptyBytes(const uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    transfer(0x00);
  }
}

#ifdef USE_ZERODMA
// Helper: Start SERCOM transaction for a queued context
// Called both initially from read()/write() and from enqueueNextPending()
bool TMC2130Stepper::startSercomTransaction(TMCAsyncContext* ctx) {
  SERCOM* sercom = SPI.getSercom();
  if (!sercom || !ctx)
    return false;

  // Transaction already setup in context, just enqueue it
  beginTransaction();
  switchCSpin(LOW);
  ctx->csActive = true;

  // Set flag and active context before enqueueSPI to prevent race:
  // ISR may fire during or immediately after enqueueSPI returns.
  _sercomEnqueued = true;
  _activeCtx = ctx;
  if (!sercom->enqueueSPI(&ctx->txn)) {
    _sercomEnqueued = false;
    switchCSpin(HIGH);
    endTransaction();
    return false;
  }

  return true;
}

// Override to actually start the next queued SPI transaction
void TMC2130Stepper::enqueueNextPending() {
  uint8_t ctxIdx;
  while (_asyncQueue.peek(ctxIdx)) {

    TMCAsyncContext* ctx = &_asyncCtxPool[ctxIdx];
    if (startSercomTransaction(ctx)) {
      return;
    }

    // Failed to start - remove from queue and continue draining.
    _asyncQueue.read(ctxIdx);
    freeContext(ctx);
  }

  _sercomEnqueued = false;
}

void TMC2130Stepper::onAsyncTxnComplete(void* user, int status) {
  if (!user) {
    return;
  }

  TMCAsyncContext* ctx = static_cast<TMCAsyncContext*>(user);
  TMC2130Stepper* self = static_cast<TMC2130Stepper*>(ctx->owner);
  if (!self) {
    return;
  }

  ctx->completionStatus = status;

  if (status != static_cast<int>(SercomSpiError::SUCCESS)) {
    self->switchCSpin(HIGH);
    self->endTransaction();
    uint8_t dummyIdx;
    self->_asyncQueue.read(dummyIdx);
    ctx->isDone = true;                // Unblock any asyncWait caller
    self->freeContext(ctx);
    self->enqueueNextPending();
    if (ctx->operation == TMCAsyncContext::READ && ctx->readCb)
      ctx->readCb(ctx->userCtx, 0, status);
    if (ctx->operation == TMCAsyncContext::WRITE && ctx->writeCb)
      ctx->writeCb(ctx->userCtx, status);
    return;
  }

  // default context configuration for both read and write
  fillZeroFrame(ctx->tx_buf);
  ctx->txn.txPtr = ctx->tx_buf;
  ctx->txn.rxPtr = ctx->rx_buf;
  ctx->txn.length = 5;
  ctx->txn.chainNext = true;

  if (ctx->operation == TMCAsyncContext::READ) {
    switch (ctx->phase) {
      case PHASE_READ_CMD:
        if (ctx->preShiftRemaining > 0) {
          ctx->phase = PHASE_SHIFT_PRE;
          ctx->preShiftRemaining--;
          return;
        }
        self->switchCSpin(HIGH);
        self->switchCSpin(LOW);
        if (ctx->postShiftRemaining > 0) {
          ctx->phase = PHASE_SHIFT_POST;
          ctx->postShiftRemaining--;
          return;
        }
        ctx->phase = PHASE_READ_DATA;
        ctx->tx_buf[0] = ctx->addressByte;
        return;

      case PHASE_SHIFT_PRE:
        if (ctx->preShiftRemaining > 0) {
          ctx->preShiftRemaining--;
          return;
        }
        self->switchCSpin(HIGH);
        self->switchCSpin(LOW);
        if (ctx->postShiftRemaining > 0) {
          ctx->phase = PHASE_SHIFT_POST;
          ctx->postShiftRemaining--;
          return;
        }
        ctx->phase = PHASE_READ_DATA;
        ctx->tx_buf[0] = ctx->addressByte;
        return;

      case PHASE_SHIFT_POST:
        if (ctx->postShiftRemaining > 0) {
          ctx->postShiftRemaining--;
          return;
        }
        ctx->phase = PHASE_READ_DATA;
        ctx->tx_buf[0] = ctx->addressByte;
        return;

      case PHASE_READ_DATA:
        ctx->txn.chainNext = false;
        ctx->status_response = ctx->rx_buf[0];
        self->status_response = ctx->status_response;
        ctx->readResult = (static_cast<uint32_t>(ctx->rx_buf[1]) << 24) |
                          (static_cast<uint32_t>(ctx->rx_buf[2]) << 16) |
                          (static_cast<uint32_t>(ctx->rx_buf[3]) << 8) |
                          (static_cast<uint32_t>(ctx->rx_buf[4]));
        self->switchCSpin(HIGH);
        self->endTransaction();
        // Copy full 5-byte frame to class-level buffer before freeing context.
        // Sync reads consume _lastRxBuf after drain-wait; async callbacks get value directly.
        memcpy(self->_lastRxBuf, ctx->rx_buf, 5);
        uint8_t dummyIdx;
        self->_asyncQueue.read(dummyIdx);  // Remove from queue
        ctx->isDone = true;                // Signal asyncWait before freeing
        self->freeContext(ctx);            // Free context slot
        self->enqueueNextPending();        // Start next queued operation
        if (ctx->readCb)
          ctx->readCb(ctx->userCtx, ctx->readResult, status);
        return;

      default:
        ctx->txn.chainNext = false;
        return;
    }
  }

  if (ctx->operation == TMCAsyncContext::WRITE) {
    switch (ctx->phase) {
      case PHASE_WRITE_CMD:
        if (ctx->preShiftRemaining > 0) {
          ctx->phase = PHASE_WRITE_SHIFT;
          ctx->preShiftRemaining--;
          return;
        }
        ctx->txn.chainNext = false;
        self->switchCSpin(HIGH);
        self->endTransaction();
        uint8_t dummyIdx1;
        self->_asyncQueue.read(dummyIdx1);  // Remove from queue
        ctx->isDone = true;                 // Signal asyncWait before freeing
        self->freeContext(ctx);             // Free context slot
        self->enqueueNextPending();         // Start next queued operation
        if (ctx->writeCb)
          ctx->writeCb(ctx->userCtx, status);
        return;

      case PHASE_WRITE_SHIFT:
        if (ctx->preShiftRemaining > 0) {
          ctx->preShiftRemaining--;
          return;
        }
        ctx->txn.chainNext = false;
        self->switchCSpin(HIGH);
        self->endTransaction();
        uint8_t dummyIdx2;
        self->_asyncQueue.read(dummyIdx2);  // Remove from queue
        ctx->isDone = true;                 // Signal asyncWait before freeing
        self->freeContext(ctx);             // Free context slot
        self->enqueueNextPending();         // Start next queued operation
        if (ctx->writeCb)
          ctx->writeCb(ctx->userCtx, status);
        return;

      default:
        ctx->txn.chainNext = false;
        return;
    }
  }
}
#endif

__attribute__((weak))
uint32_t TMC2130Stepper::read(uint8_t addressByte,
                               void (*onComplete)(void* user, uint32_t value, int status),
                               void* user) {
#ifdef USE_ZERODMA
  SERCOM* sercom = SPI.getSercom();
  if (sercom == nullptr || TMC_SW_SPI != nullptr) {
    if (onComplete)
      onComplete(user, 0, static_cast<int>(SercomSpiError::UNKNOWN_ERROR));
    return 0;
  }

  TMCAsyncContext* ctx = allocateContext();
  if (!ctx) {
    if (onComplete)
      onComplete(user, 0, static_cast<int>(SercomSpiError::UNKNOWN_ERROR));
    return 0;
  }

  // Setup async context
  ctx->operation = TMCAsyncContext::READ;
  ctx->phase = PHASE_READ_CMD;
  ctx->addressByte = addressByte;
  ctx->link_index = link_index;
  ctx->chain_length = chain_length;
  ctx->preShiftRemaining = (link_index > 1) ? (link_index - 1) : 0;
  int8_t iAfterPreShift = (link_index > 1) ? link_index : 1;
  ctx->postShiftRemaining = (chain_length > iAfterPreShift) ? (chain_length - iAfterPreShift) : 0;
  ctx->readCb = onComplete;
  ctx->writeCb = nullptr;
  ctx->userCtx = user;
  ctx->owner = this;
  ctx->readResult = 0;
  ctx->status_response = 0;

  fillZeroFrame(ctx->tx_buf);
  ctx->tx_buf[0] = addressByte;

  ctx->txn.txPtr = ctx->tx_buf;
  ctx->txn.rxPtr = ctx->rx_buf;
  ctx->txn.length = 5;
  ctx->txn.onComplete = &TMC2130Stepper::onAsyncTxnComplete;
  ctx->txn.user = ctx;
  ctx->txn.chainNext = false;

  uint8_t ctxIdx = static_cast<uint8_t>(ctx - _asyncCtxPool);

  if (!_asyncQueue.store(ctxIdx)) {
    freeContext(ctx);
    if (onComplete)
      onComplete(user, 0, static_cast<int>(SercomSpiError::UNKNOWN_ERROR));
    return 0;
  }

  // If no operation currently active, start this one immediately
  if (!_sercomEnqueued) {
    if (!startSercomTransaction(ctx)) {
      uint8_t dummyIdx;
      _asyncQueue.read(dummyIdx);
      freeContext(ctx);
      if (onComplete)
        onComplete(user, 0, static_cast<int>(SercomSpiError::UNKNOWN_ERROR));
      return 0;
    }
  }

  if (!onComplete) {
    asyncWait();
    return (static_cast<uint32_t>(_lastRxBuf[1]) << 24) |
           (static_cast<uint32_t>(_lastRxBuf[2]) << 16) |
           (static_cast<uint32_t>(_lastRxBuf[3]) << 8) |
           (static_cast<uint32_t>(_lastRxBuf[4]));
  }

  return 0;
#else
  uint32_t out = 0UL;
  int8_t i = 1;

  beginTransaction();
  switchCSpin(LOW);
  transfer(addressByte);
  // Clear SPI
  transferEmptyBytes(4);

  // Shift the written data to the correct driver in chain
  // Default link_index = -1 and no shifting happens
  while(i < link_index) {
    transferEmptyBytes(5);
    i++;
  }

  switchCSpin(HIGH);
  switchCSpin(LOW);

  // Shift data from target link into the last one...
  while(i < chain_length) {
    transferEmptyBytes(5);
    i++;
  }

  // ...and once more to MCU
  status_response = transfer(addressByte); // Send the address byte again
  out  = transfer(0x00);
  out <<= 8;
  out |= transfer(0x00);
  out <<= 8;
  out |= transfer(0x00);
  out <<= 8;
  out |= transfer(0x00);

  endTransaction();
  switchCSpin(HIGH);

  return out;
#endif
}

__attribute__((weak))
void TMC2130Stepper::write(uint8_t addressByte, uint32_t config,
                            void (*onComplete)(void* user, int status),
                            void* user) {
#ifdef USE_ZERODMA
  // ========================================
  // SERCOM DMA Path (USE_ZERODMA defined)
  // Writes are always fire-and-forget (async).
  // Back-pressure: if pool or queue is full, spin-wait for a slot.
  // ========================================
  SERCOM* sercom = SPI.getSercom();
  if (sercom == nullptr || TMC_SW_SPI != nullptr) {
    if (onComplete)
      onComplete(user, static_cast<int>(SercomSpiError::UNKNOWN_ERROR));
    return;
  }

  TMCAsyncContext* ctx = allocateContext();
  if (!ctx) {
    if (onComplete)
      onComplete(user, static_cast<int>(SercomSpiError::UNKNOWN_ERROR));
    return;
  }

  // Setup async context
  ctx->operation = TMCAsyncContext::WRITE;
  ctx->phase = PHASE_WRITE_CMD;
  ctx->addressByte = addressByte | TMC_WRITE;
  ctx->configValue = config;
  ctx->link_index = link_index;
  ctx->chain_length = chain_length;
  ctx->preShiftRemaining = (link_index > 1) ? (link_index - 1) : 0;
  ctx->postShiftRemaining = 0;
  ctx->readCb = nullptr;
  ctx->writeCb = onComplete;
  ctx->userCtx = user;
  ctx->owner = this;

  ctx->tx_buf[0] = ctx->addressByte;
  ctx->tx_buf[1] = (config >> 24) & 0xFF;
  ctx->tx_buf[2] = (config >> 16) & 0xFF;
  ctx->tx_buf[3] = (config >> 8) & 0xFF;
  ctx->tx_buf[4] = (config >> 0) & 0xFF;

  ctx->txn.txPtr = ctx->tx_buf;
  ctx->txn.rxPtr = ctx->rx_buf;
  ctx->txn.length = 5;
  ctx->txn.onComplete = &TMC2130Stepper::onAsyncTxnComplete;
  ctx->txn.user = ctx;
  ctx->txn.chainNext = false;

  uint8_t ctxIdx = static_cast<uint8_t>(ctx - _asyncCtxPool);

  if (!_asyncQueue.store(ctxIdx)) {
    freeContext(ctx);
    if (onComplete)
      onComplete(user, static_cast<int>(SercomSpiError::UNKNOWN_ERROR));
    return;
  }

  // If no operation currently active, start this one immediately
  if (!_sercomEnqueued) {
    if (!startSercomTransaction(ctx)) {
      uint8_t dummyIdx;
      _asyncQueue.read(dummyIdx);
      freeContext(ctx);
      if (onComplete)
        onComplete(user, static_cast<int>(SercomSpiError::UNKNOWN_ERROR));
      return;
    }
  }

  return;

#else
  // ========================================
  // Synchronous Blocking Path (default if USE_ZERODMA not defined)
  // ========================================
  addressByte |= TMC_WRITE;
  int8_t i = 1;

  beginTransaction();
  switchCSpin(LOW);
  status_response = transfer(addressByte);
  transfer(config>>24);
  transfer(config>>16);
  transfer(config>>8);
  transfer(config);

  // Shift the written data to the correct driver in chain
  // Default link_index = -1 and no shifting happens
  while(i < link_index) {
    transferEmptyBytes(5);
    i++;
  }

  endTransaction();
  switchCSpin(HIGH);
  
  // If user provided a callback, invoke it after completion
  if (onComplete) {
    onComplete(user, 0);  // status=0 (success)
  }
#endif
}

void TMC2130Stepper::begin() {
  //set pins
  pinMode(_pinCS, OUTPUT);
  switchCSpin(HIGH);

  if (TMC_SW_SPI != nullptr) TMC_SW_SPI->init();

  GCONF(GCONF_register.sr);
  CHOPCONF(CHOPCONF_register.sr);
  COOLCONF(COOLCONF_register.sr);
  PWMCONF(PWMCONF_register.sr);
  IHOLD_IRUN(IHOLD_IRUN_register.sr);

  toff(8); //off_time(8);
  tbl(1); //blank_time(24);
}

/**
 *  Helper functions
 */

bool TMC2130Stepper::isEnabled() { return !drv_enn_cfg6() && toff(); }

void TMC2130Stepper::push() {
  GCONF(GCONF_register.sr);
  IHOLD_IRUN(IHOLD_IRUN_register.sr);
  TPOWERDOWN(TPOWERDOWN_register.sr);
  TPWMTHRS(TPWMTHRS_register.sr);
  TCOOLTHRS(TCOOLTHRS_register.sr);
  THIGH(THIGH_register.sr);
  XDIRECT(XDIRECT_register.sr);
  VDCMIN(VDCMIN_register.sr);
  CHOPCONF(CHOPCONF_register.sr);
  COOLCONF(COOLCONF_register.sr);
  DCCTRL(DCCTRL_register.sr);
  PWMCONF(PWMCONF_register.sr);
  ENCM_CTRL(ENCM_CTRL_register.sr);
}

///////////////////////////////////////////////////////////////////////////////////////
// R: IOIN
uint32_t  TMC2130Stepper::IOIN()    { return read(IOIN_t::address); }
bool TMC2130Stepper::step()         { IOIN_t r{0}; r.sr = IOIN(); return r.step; }
bool TMC2130Stepper::dir()          { IOIN_t r{0}; r.sr = IOIN(); return r.dir; }
bool TMC2130Stepper::dcen_cfg4()    { IOIN_t r{0}; r.sr = IOIN(); return r.dcen_cfg4; }
bool TMC2130Stepper::dcin_cfg5()    { IOIN_t r{0}; r.sr = IOIN(); return r.dcin_cfg5; }
bool TMC2130Stepper::drv_enn_cfg6() { IOIN_t r{0}; r.sr = IOIN(); return r.drv_enn_cfg6; }
bool TMC2130Stepper::dco()          { IOIN_t r{0}; r.sr = IOIN(); return r.dco; }
uint8_t TMC2130Stepper::version()   { IOIN_t r{0}; r.sr = IOIN(); return r.version; }
///////////////////////////////////////////////////////////////////////////////////////
// W: TCOOLTHRS
uint32_t TMC2130Stepper::TCOOLTHRS() { return TCOOLTHRS_register.sr; }
void TMC2130Stepper::TCOOLTHRS(uint32_t input) {
  TCOOLTHRS_register.sr = input;
  write(TCOOLTHRS_register.address, TCOOLTHRS_register.sr);
}
///////////////////////////////////////////////////////////////////////////////////////
// W: THIGH
uint32_t TMC2130Stepper::THIGH() { return THIGH_register.sr; }
void TMC2130Stepper::THIGH(uint32_t input) {
  THIGH_register.sr = input;
  write(THIGH_register.address, THIGH_register.sr);
}
///////////////////////////////////////////////////////////////////////////////////////
// RW: XDIRECT
uint32_t TMC2130Stepper::XDIRECT() {
  return read(XDIRECT_register.address);
}
void TMC2130Stepper::XDIRECT(uint32_t input) {
  XDIRECT_register.sr = input;
  write(XDIRECT_register.address, XDIRECT_register.sr);
}
void TMC2130Stepper::coil_A(int16_t B)  { XDIRECT_register.coil_A = B; write(XDIRECT_register.address, XDIRECT_register.sr); }
void TMC2130Stepper::coil_B(int16_t B)  { XDIRECT_register.coil_B = B; write(XDIRECT_register.address, XDIRECT_register.sr); }
int16_t TMC2130Stepper::coil_A()        { XDIRECT_t r{0}; r.sr = XDIRECT(); return r.coil_A; }
int16_t TMC2130Stepper::coil_B()        { XDIRECT_t r{0}; r.sr = XDIRECT(); return r.coil_B; }
///////////////////////////////////////////////////////////////////////////////////////
// W: VDCMIN
uint32_t TMC2130Stepper::VDCMIN() { return VDCMIN_register.sr; }
void TMC2130Stepper::VDCMIN(uint32_t input) {
  VDCMIN_register.sr = input;
  write(VDCMIN_register.address, VDCMIN_register.sr);
}
///////////////////////////////////////////////////////////////////////////////////////
// RW: DCCTRL
void TMC2130Stepper::DCCTRL(uint32_t input) {
	DCCTRL_register.sr = input;
	write(DCCTRL_register.address, DCCTRL_register.sr);
}
void TMC2130Stepper::dc_time(uint16_t input) {
	DCCTRL_register.dc_time = input;
	write(DCCTRL_register.address, DCCTRL_register.sr);
}
void TMC2130Stepper::dc_sg(uint8_t input) {
	DCCTRL_register.dc_sg = input;
	write(DCCTRL_register.address, DCCTRL_register.sr);
}

uint32_t TMC2130Stepper::DCCTRL() {
	return read(DCCTRL_register.address);
}
uint16_t TMC2130Stepper::dc_time() {
	DCCTRL_t r{0};
  r.sr = DCCTRL();
	return r.dc_time;
}
uint8_t TMC2130Stepper::dc_sg() {
	DCCTRL_t r{0};
  r.sr = DCCTRL();
	return r.dc_sg;
}
///////////////////////////////////////////////////////////////////////////////////////
// R: PWM_SCALE
uint8_t TMC2130Stepper::PWM_SCALE() { return read(PWM_SCALE_t::address); }
///////////////////////////////////////////////////////////////////////////////////////
// W: ENCM_CTRL
uint8_t TMC2130Stepper::ENCM_CTRL() { return ENCM_CTRL_register.sr; }
void TMC2130Stepper::ENCM_CTRL(uint8_t input) {
  ENCM_CTRL_register.sr = input;
  write(ENCM_CTRL_register.address, ENCM_CTRL_register.sr);
}
void TMC2130Stepper::inv(bool B)      { ENCM_CTRL_register.inv = B;       write(ENCM_CTRL_register.address, ENCM_CTRL_register.sr); }
void TMC2130Stepper::maxspeed(bool B) { ENCM_CTRL_register.maxspeed  = B; write(ENCM_CTRL_register.address, ENCM_CTRL_register.sr); }
bool TMC2130Stepper::inv()            { return ENCM_CTRL_register.inv; }
bool TMC2130Stepper::maxspeed()       { return ENCM_CTRL_register.maxspeed; }
///////////////////////////////////////////////////////////////////////////////////////
// R: LOST_STEPS
uint32_t TMC2130Stepper::LOST_STEPS() { return read(LOST_STEPS_t::address); }

void TMC2130Stepper::sg_current_decrease(uint8_t value) {
  switch(value) {
    case 32: sedn(0b00); break;
    case  8: sedn(0b01); break;
    case  2: sedn(0b10); break;
    case  1: sedn(0b11); break;
  }
}
uint8_t TMC2130Stepper::sg_current_decrease() {
  switch(sedn()) {
    case 0b00: return 32;
    case 0b01: return  8;
    case 0b10: return  2;
    case 0b11: return  1;
  }
  return 0;
}
