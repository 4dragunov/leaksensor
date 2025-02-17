#include <stdbool.h>
#include <stdint.h>
#include <stm32f1xx.h>
#include "eeprom.h"
#include "nonvol.h"
#include "utilities.h"
#include "cmsis_os.h"

extern uint32_t _emulated_eeprom_start[];
extern uint32_t _EEPROM_PAGES[];

#define PAGE_SIZE        (uint32_t)FLASH_PAGE_SIZE  /* Page size */
#define PAGES            ((uint32_t)_EEPROM_PAGES)
#define PAGE_0_ADDRESS   ((uint32_t)_emulated_eeprom_start)
#define PAGE_1_ADDRESS   ((uint32_t)_emulated_eeprom_start + (PAGES * PAGE_SIZE))

#define FLASH_READ(address) static_cast<uint32_t>(*(__IO uint32_t*)address)

/* Variables  */

uint32_t Eeprom::pageAddress[Eeprom::PageIdx::PAGES_NUM] = {PAGE_0_ADDRESS, PAGE_1_ADDRESS};
Eeprom::PageState Eeprom::pageStates[Eeprom::PageIdx::PAGES_NUM] = {Eeprom::PageState::CLEAR, Eeprom::PageState::CLEAR};

bool Eeprom::mInitialized = false;
const Eeprom::address Eeprom::base = 0;          // start address of first record in EEPROM
const Eeprom::address Eeprom::pages = PAGES;
const Eeprom::address Eeprom::end =  (((PAGES * PAGE_SIZE) - sizeof(Eeprom::page_state_record)) / sizeof(Eeprom::data_record));           // end of available EEPROM
const Eeprom::address Eeprom::size = Eeprom::end - Eeprom::base;
Eeprom::address Eeprom::free = Eeprom::size;

size_t Eeprom::erase_time = 0;
size_t Eeprom::read_time = 0;
size_t Eeprom::write_time = 0;
size_t Eeprom::transfer_time = 0;

const Eeprom::page_state_record Eeprom::pageStateValues[PageState::STATE_COUNT - 1] = { std::numeric_limits<Eeprom::page_state_record>::max(),
											std::numeric_limits<Eeprom::page_state_record>::min(),
											std::numeric_limits<Eeprom::page_state_record>::max()/0x03};
const char* Eeprom::pageStateNames[STATE_COUNT] = {"Clear", "Active", "Receiving", "Undefined"};

osMutexDef (RwLock);

/******************************************************************************/
Eeprom::Eeprom():mRwLock(osMutexNew(osMutex(RwLock)))

{
	if(!mInitialized) {
		mInitialized = (Init() == Result::OK);
	}
}

/******************************************************************************/
Eeprom::~Eeprom() {
	if (mRwLock)  {
	    // assert(osMutexDelete(mRwLock) == osOK);
	}
}

/******************************************************************************/
Eeprom& Eeprom::Instance()
{
   volatile static Eeprom s;
   return const_cast<Eeprom&>(s);
}
/******************************************************************************/
Eeprom::Result Eeprom::read(const Eeprom::address addr, Eeprom::data* buf, size_t length){


 //	  assert_param(addr < this->size);
 //	  assert_param(buf);
 //	  assert_param(length > 0);
   if( mInitialized )
   {
	    Eeprom::address i = 0;

		do
		{
			if(read(addr + i, &buf[i]) != Result::OK) {
				return Eeprom::Result::ERROR;;
			}
			i++;
		} while(--length);
		return Result::OK;
   }else
	  return Eeprom::Result::ERROR;;
}

/******************************************************************************/
Eeprom::Result Eeprom::write(const Eeprom::address addr, const Eeprom::data* buf, size_t length){
	//	  assert_param(addr < this->size);
	//	  assert_param(buf);
	//	  assert_param(length > 0);
   if( Eeprom::mInitialized )
   {
	   Eeprom::address i = 0;

	   do
	   {
		   if (write((uint32_t)addr + i, buf[i]) != Result::OK){

				return Eeprom::Result::ERROR;
		   }
		   i++;
	   }while(--length);
	   return Result::OK;
   }
   else
	   return Eeprom::Result::ERROR;
}

/******************************************************************************/
Eeprom::Result Eeprom::WriteRecord(const uint32_t address, const Eeprom::address varId, const Eeprom::data varValue)
{
  Eeprom::Result res = Result::OK;
  HAL_StatusTypeDef flashRes = HAL_OK;
  Eeprom::data_record dataRecord = static_cast<Eeprom::data_record>((Eeprom::data)varValue << std::numeric_limits<typename Eeprom::address>::digits) | (Eeprom::address)varId;

  HAL_FLASH_Unlock();
  flashRes = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, dataRecord);
  HAL_FLASH_Lock();
  if (flashRes != HAL_OK)
  {
    res = Result::ERROR;
  }
  return res;
}

/******************************************************************************/
Eeprom::Result Eeprom::read(const Eeprom::address varId, Eeprom::data *varValue)
{
  Eeprom::Result res = Result::ERROR;
  PageIdx activePage = PageIdx::PAGE_0;

  osMutexWait(mRwLock, osWaitForever);
  if (GetActivePageIdx(&activePage) == Result::OK)
  {
	  auto read_time_start = osKernelSysTick();
	  uint32_t startAddr = pageAddress[activePage] + sizeof(Eeprom::page_state_record);
	  uint32_t endAddr = pageAddress[activePage] + (PAGES * PAGE_SIZE) - sizeof(Eeprom::data_record);
	  uint32_t addr = endAddr;
	  Eeprom::address empty = 0;

	  while (addr >= startAddr)
	  {
		data_record idData = FLASH_READ(addr);

		Eeprom::address rvid = (idData & std::numeric_limits<Eeprom::address>::max());
		if (rvid == varId)
		{
		  *varValue = (idData >> std::numeric_limits<Eeprom::address>::digits);
		  res =  Result::OK;
		  free = empty;
		  read_time = osKernelSysTick() - read_time_start;
		  break;
		}
		else
		{
		  addr -= sizeof(Eeprom::data_record);
		  if(rvid == std::numeric_limits<Eeprom::address>::max())
			  empty++;
		}
	  }
  }
  osMutexRelease(mRwLock);
  return  res;
}

/******************************************************************************/
Eeprom::Result Eeprom::PageTransfer(const PageIdx activePage, const Eeprom::address eepromAddress, Eeprom::data eepromData)
{
  Eeprom::Result res = Result::OK;
  PageIdx oldPage, newPage;
  auto start = osKernelSysTick();
  oldPage = activePage;
  newPage = (PageIdx)(1 - activePage);

  res = SetPageState(newPage, PageState::RECEIVING_DATA); // write status to the begining of the page

  if (res != Result::OK)
  {
    return res;
  }

  uint32_t recordAddr = pageAddress[newPage] + sizeof(Eeprom::page_state_record);//first record
  res = WriteRecord(recordAddr, eepromAddress, eepromData);

  if (res != Result::OK)
  {
    return res;
  }

  recordAddr += sizeof(Eeprom::data_record);

  for (Eeprom::address eepAddr = 0; eepAddr < Eeprom::size; eepAddr++)
  {
    if (eepAddr != eepromAddress) //skip new data record as already written above
    {
      Eeprom::data eepData = 0;
      res = read(eepAddr, &eepData);

      if (res == Result::OK)
      {
        res = WriteRecord(recordAddr, eepAddr, eepData);

        if (res != Result::OK)
        {
          return res;
        }

        recordAddr += sizeof(Eeprom::data_record);
      }
    }
  }

  res = ClearPage(oldPage);

  if (res != Result::OK)
  {
    return res;
  }

  res = SetPageState(newPage, PageState::ACTIVE);
  transfer_time = osKernelSysTick() - start;
  return res;
}

/******************************************************************************/
Eeprom::Result Eeprom::write(const Eeprom::address eepromAddress, const Eeprom::data data)
{
  Eeprom::Result res = Result::ERROR;
  Eeprom::address occuped = 0;
  Eeprom::data available = 0;
  PageIdx activePage = PageIdx::PAGE_0;

  osMutexWait(mRwLock, osWaitForever);
  if (GetActivePageIdx(&activePage) == Result::OK)
  {
	  auto write_time_start = osKernelSysTick();
	  res = read(eepromAddress, &available);

	  if((res == Result::OK && available != data) ||  res == Result::ERROR) {//data not the same value or not found

		  uint32_t startAddr = pageAddress[activePage] +  sizeof(Eeprom::page_state_record);
		  uint32_t endAddr = pageAddress[activePage] + (PAGES * PAGE_SIZE) - sizeof(Eeprom::data_record); //points to the last record
		  uint32_t addr = startAddr;

		  while (addr <= endAddr)
		  {
			uint32_t idData = FLASH_READ(addr);
			if (idData == pageStateValues[PageState::CLEAR])
			{
				res =  WriteRecord(addr, eepromAddress, data);
				free = end - occuped - 1;
				write_time = osKernelSysTick() - write_time_start;
				osMutexRelease(mRwLock);
				return res;
			}
			else
			{
			  addr += sizeof(Eeprom::data_record);
			  occuped++;
			}
		  }
		  res = PageTransfer(activePage, eepromAddress, data);
	  }
  }
  osMutexRelease(mRwLock);
  return res;
}

/******************************************************************************/
Eeprom::Result Eeprom::RestorePageData(const PageIdx oldPage, const PageIdx newPage)
{
	Eeprom::Result res = Result::OK;
	DBG("eeprom %s old:%i new:%i\n",__FUNCTION__, oldPage, newPage);
	Eeprom::data_record last_record = FLASH_READ(pageAddress[newPage] + sizeof(Eeprom::page_state_record));
	Eeprom::address eep_address = (last_record & std::numeric_limits<Eeprom::address>::max());
	Eeprom::data eep_data = (last_record >> std::numeric_limits<Eeprom::address>::digits);
	res = ClearPage(newPage);
	if((res == Result::OK) && (eep_address != (last_record & std::numeric_limits<Eeprom::address>::max())))
		res = PageTransfer(oldPage, eep_address, eep_data);
	return res;
}

/******************************************************************************/
Eeprom::PageState Eeprom::ReadPageState(const Eeprom::PageIdx idx)
{
  auto stateValue = FLASH_READ(pageAddress[idx]);
  for (uint8_t pageState = 0; pageState < STATE_COUNT - 1; pageState++)
	  if(pageStateValues[pageState] == stateValue)
		  return static_cast<Eeprom::PageState>(pageState);
  return Eeprom::PageState::UNDEFINED;
}

/******************************************************************************/
Eeprom::Result Eeprom::SetPageState(const Eeprom::PageIdx idx, const Eeprom::PageState state)
{
  Eeprom::Result res = Result::OK;
  HAL_StatusTypeDef flashRes = HAL_OK;
  DBG("eeprom %s page:%i state:%s\n",__FUNCTION__, idx, pageStateNames[state]);
  assert(state < Eeprom::PageState::UNDEFINED);
  HAL_FLASH_Unlock();
  flashRes = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, pageAddress[idx], pageStateValues[state]);
  HAL_FLASH_Lock();

  if (flashRes != HAL_OK)
  {
	res = Result::ERROR;
  }else
	pageStates[idx] = state;
  return res;
}

/******************************************************************************/
Eeprom::Result Eeprom::ClearPage(const Eeprom::PageIdx idx)
{
  Eeprom::Result res = Result::OK;

  FLASH_EraseInitTypeDef erase;
  DBG("eeprom %s page:%i\n",__FUNCTION__, idx);
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.Banks = FLASH_BANK_1;
  erase.PageAddress = pageAddress[idx];
  erase.NbPages = PAGES;

  HAL_StatusTypeDef flashRes = HAL_OK;
  uint32_t pageError = 0;

  HAL_FLASH_Unlock();
  flashRes = HAL_FLASHEx_Erase(&erase, &pageError);
  HAL_FLASH_Lock();

  if (flashRes != HAL_OK)
  {
	DBG("pageError %li\n",pageError);
    res = Result::ERROR;
    return res;
  }
  return SetPageState(idx, CLEAR);
}

/******************************************************************************/
Eeprom::Result Eeprom::erase()
{
  Eeprom::Result res = Result::OK;
  DBG("eeprom %s\n",__FUNCTION__);
  auto start = osKernelSysTick();
  for (uint8_t i = 0; i < PAGES_NUM; i++)
  {
    res = ClearPage((PageIdx)i);
    if (res != Result::OK)
    {
      return res;
    }
  }
  res = SetPageState(PAGE_0, ACTIVE);
  erase_time = osKernelSysTick() - start;
  return res;
}

/******************************************************************************/
Eeprom::Result Eeprom::GetActivePageIdx(Eeprom::PageIdx *idx)
{
  Eeprom::Result res = Result::OK;

  for (uint8_t i = 0; i < PAGES_NUM; i++)
  {
    pageStates[i] = ReadPageState((PageIdx)i);
  }

  if ((pageStates[PAGE_0] == ACTIVE) && (pageStates[PAGE_1] != ACTIVE))
  {
    *idx = PAGE_0;
  }
  else
  {
    if ((pageStates[PAGE_1] == ACTIVE) && (pageStates[PAGE_0] != ACTIVE))
    {
      *idx = PAGE_1;
    }
    else
    {
      res = Result::ERROR;
    }
  }

  return res;
}

/******************************************************************************/
Eeprom::Result Eeprom::Init()
{
  Eeprom::Result res = Result::OK;
  DBG("eeprom %s\n",__FUNCTION__);

  for (uint8_t i = 0; i < PAGES_NUM; i++)
  {
	pageStates[i] = ReadPageState((PageIdx)i);
	DBG("eeprom page:%i state:%s\n", i, pageStateNames[i]);
  }

  if (((pageStates[PAGE_0] == CLEAR) && (pageStates[PAGE_1] == CLEAR)) ||
      ((pageStates[PAGE_0] == ACTIVE) && (pageStates[PAGE_1] == ACTIVE)) ||
      ((pageStates[PAGE_0] == RECEIVING_DATA) && (pageStates[PAGE_1] == RECEIVING_DATA)))
  {
    res = erase();
    if (res != Result::OK)
    {
      return res;
    }
    res = SetPageState(PAGE_0, ACTIVE);
  }else
  if ((pageStates[PAGE_0] == RECEIVING_DATA) && (pageStates[PAGE_1] == CLEAR))
  {
    res = SetPageState(PAGE_0, ACTIVE);
  }else
  if ((pageStates[PAGE_0] == CLEAR) && (pageStates[PAGE_1] == RECEIVING_DATA))
  {
	res = SetPageState(PAGE_1, ACTIVE);
  }else
  if ((pageStates[PAGE_0] == RECEIVING_DATA) && (pageStates[PAGE_1] == ACTIVE))
  {
	res = RestorePageData(PAGE_1, PAGE_0);
  }else
  if ((pageStates[PAGE_0] == ACTIVE) && (pageStates[PAGE_1] == RECEIVING_DATA))
  {
	res = RestorePageData(PAGE_0, PAGE_1);
  }
  return res;
}
/******************************************************************************/
extern "C" bool eepromInit()
{
	return Eeprom::Instance().initialized();
}
/******************************************************************************/
extern "C" bool eepromWrite(const uint16_t addr, const uint8_t* buf, const size_t length)
{
	return Eeprom::Instance().write(addr, (Eeprom::data*)buf, length) == Eeprom::Result::OK;
}
/******************************************************************************/
extern "C" bool eepromRead(const uint16_t addr, uint8_t* buf, const size_t length)
{
	return Eeprom::Instance().read(addr, (Eeprom::data*)buf, length) == Eeprom::Result::OK;
}
