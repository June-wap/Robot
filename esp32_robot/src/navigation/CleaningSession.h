#ifndef CLEANING_SESSION_H
#define CLEANING_SESSION_H

bool buildPlan();
void startCleaning();
void finishCleaning();
void stopAll();
void setError(const char* m);

#endif // CLEANING_SESSION_H
