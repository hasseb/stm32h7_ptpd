#include "ptpd.h"

#include <string.h>

#if LWIP_PTPD

// Static array of PTPD timers.
static osTimerId ptpd_timer_id[TIMER_ARRAY_SIZE];
static bool ptpd_timers_expired[TIMER_ARRAY_SIZE];

// Callback for timers.
static void ptpd_timer_callback(void *arg)
{
  // Sanity check the index.
  for (uint8_t i=0; i < TIMER_ARRAY_SIZE; i++)
  {
    if ((osTimerId)arg == ptpd_timer_id[i])
    {
      // Mark the indicated timer as expired.
      ptpd_timers_expired[i] = true;

      // Notify the PTP thread of a pending operation.
      ptpd_alert();
    }
  }
}

// Initialize PTPD timers.
osTimerDef(PDELAYREQ_INTERVAL_TIMER, ptpd_timer_callback);
osTimerDef(DELAYREQ_INTERVAL_TIMER, ptpd_timer_callback);
osTimerDef(SYNC_INTERVAL_TIMER, ptpd_timer_callback);
osTimerDef(ANNOUNCE_RECEIPT_TIMER, ptpd_timer_callback);
osTimerDef(ANNOUNCE_INTERVAL_TIMER, ptpd_timer_callback);
osTimerDef(QUALIFICATION_TIMEOUT, ptpd_timer_callback);
void ptpd_timer_init(void)
{
  memset(ptpd_timers_expired, false, sizeof(ptpd_timers_expired));

  ptpd_timer_id[PDELAYREQ_INTERVAL_TIMER] = osTimerCreate(osTimer(PDELAYREQ_INTERVAL_TIMER), osTimerPeriodic, &ptpd_timers_expired[PDELAYREQ_INTERVAL_TIMER]);
  ptpd_timer_id[DELAYREQ_INTERVAL_TIMER] = osTimerCreate(osTimer(DELAYREQ_INTERVAL_TIMER), osTimerPeriodic, &ptpd_timers_expired[DELAYREQ_INTERVAL_TIMER]);
  ptpd_timer_id[SYNC_INTERVAL_TIMER] = osTimerCreate(osTimer(SYNC_INTERVAL_TIMER), osTimerPeriodic, &ptpd_timers_expired[SYNC_INTERVAL_TIMER]);
  ptpd_timer_id[ANNOUNCE_RECEIPT_TIMER] = osTimerCreate(osTimer(ANNOUNCE_RECEIPT_TIMER), osTimerPeriodic, &ptpd_timers_expired[ANNOUNCE_RECEIPT_TIMER]);
  ptpd_timer_id[ANNOUNCE_INTERVAL_TIMER] = osTimerCreate(osTimer(ANNOUNCE_INTERVAL_TIMER), osTimerPeriodic, &ptpd_timers_expired[ANNOUNCE_INTERVAL_TIMER]);
  ptpd_timer_id[QUALIFICATION_TIMEOUT] = osTimerCreate(osTimer(QUALIFICATION_TIMEOUT), osTimerPeriodic, &ptpd_timers_expired[QUALIFICATION_TIMEOUT]);

  // int32_t i;

  // // Create the various timers used in the system.
  // for (i = 0; i < TIMER_ARRAY_SIZE; i++)
  // {
  //   // Mark the timer as not expired.
  //   ptpd_timers_expired[i] = false;

  //   // Create the timer.
  //   ptpd_timer_id[i] = osTimerCreate(osTimer(i), osTimerPeriodic, &ptpd_timers_expired[i]);
  // }
}

// Start the indexed timer with the given interval.
void ptpd_timer_start(int32_t index, uint32_t interval_ms)
{
  // Sanity check the index.
  if (index >= TIMER_ARRAY_SIZE)
    return;
  if (ptpd_timer_id[index] == 0)
    return;

  DBGV("PTPD: set timer %d to %u\n", index, interval_ms);

  // Reset the timer expired flag.
  ptpd_timers_expired[index] = false;

  // Start the timer with the specified duration.
  osTimerStart(ptpd_timer_id[index], interval_ms);
}

// Stop the indexed timer.
void ptpd_timer_stop(int32_t index)
{
  // Sanity check the index.
  if (index >= TIMER_ARRAY_SIZE)
    return;

  DBGV("PTPD: stop timer %d\n", index);

  // Stop the timer.
  osTimerStop(ptpd_timer_id[index]);

  // Reset the expired flag.
  ptpd_timers_expired[index] = false;
}

// If the timer has expired, this function will reset the
// expired flag and return true, otherwise it will false.
bool ptpd_timer_expired(int32_t index)
{
  // Sanity check the index.
  if (index >= TIMER_ARRAY_SIZE)
    return false;

#if 0
  DBGV("PTPD: timer %d %s\n", index,
       ptpd_timers_expired[index] ? "expired" : "not expired");
#endif

  // Return false if the timer hasn't expired.
  if (!ptpd_timers_expired[index])
    return false;

  // We only return the timer expired once.
  ptpd_timers_expired[index] = false;

  // Return true since the timer expired.
  return true;
}

#endif // LWIP_PTPD
