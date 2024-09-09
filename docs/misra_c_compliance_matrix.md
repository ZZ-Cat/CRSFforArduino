# MISRA C Compliance and Deviations

This document outlines the MISRA C compliance of CRSF for Arduino and documents any deviations from the MISRA C guidelines.  
The [compliance](#compliance) section lists all of the MISRA C guidelines that CRSF for Arduino is compliant with.
The [deviations](#deviations) section lists any deviations from the MISRA C guidelines.

## Compliance

The following table is the MISRA C compliance matrix for CRSF for Arduino.  
Each row corresponds to a MISRA C guideline, and each column corresponds to a tool or method used to ensure compliance with the guideline.  
A checkmark :white_check_mark: in a cell indicates that the guideline is checked by the corresponding tool or method.  
A cross :x: in a cell indicates that the guideline is not checked by the corresponding tool or method.  

If a cross is present in any column that corresponds to either the GCC Arm None EABI compiler or Cppcheck static analysis tool, this means that checking the guideline is not possible with that tool or method because it is not supported.  
In this case, the guideline must be checked manually during code review, as indicated by a checkmark in the "Manual review" column.

**Legend:**

- **Advisory**: CRSF for Arduino follows this guideline wherever reasonable and practical, unless the relevant deviation is documented.
- **Required**: CRSF for Arduino must follow this guideline, unless the relevant deviation is documented.
- **Mandatory**: CRSF for Arduino must follow this guideline. No deviations are permitted.

| MISRA Rule | Rule flag | GCC Arm None EABI | Cppcheck | Manual review |
| --- | --- | --- | --- | --- |
| Rule 1.1 | Required | :x: | :x: | :white_check_mark: |
| Rule 1.2 | Required | :x: | :x: | :white_check_mark: |
| Rule 1.3 | Required | :x: | :white_check_mark: | :x: |
| Rule 2.1 | Required | :x: | :white_check_mark: | :x: |
| Rule 2.2 | Required | :x: | :white_check_mark: | :x: |
| Rule 2.3 | Advisory | :x: | :white_check_mark: | :x: |
| Rule 2.4 | Advisory | :x: | :white_check_mark: | :x: |
| Rule 2.5 | Advisory | :x: | :white_check_mark: | :x: |
| Rule 2.6 | Advisory | :x: | :white_check_mark: | :x: |
| Rule 2.7 | Advisory | :x: | :white_check_mark: | :x: |
| Rule 3.1 | Required | :x: | :white_check_mark: | :x: |
| Rule 3.2 | Required | :x: | :white_check_mark: | :x: |
| Rule 4.1 | Required | :x: | :white_check_mark: | :x: |
| Rule 4.2 | Required | :x: | :white_check_mark: | :x: |
| Rule 5.1 | Required | :x: | :white_check_mark: | :x: |
| Rule 5.2 | Required | :x: | :white_check_mark: | :x: |
| Rule 5.3 | Required | :x: | :white_check_mark: | :x: |
| Rule 5.4 | Required | :x: | :white_check_mark: | :x: |
| Rule 5.5 | Required | :x: | :white_check_mark: | :x: |
| Rule 5.6 | Required | :x: | :white_check_mark: | :x: |
| Rule 5.7 | Required | :x: | :white_check_mark: | :x: |
| Rule 5.8 | Required | :x: | :white_check_mark: | :x: |
| Rule 5.9 | Required | :x: | :white_check_mark: | :x: |
| Rule 6.1 | Required | :x: | :white_check_mark: | :x: |
| Rule 6.2 | Required | :x: | :white_check_mark: | :x: |
| Rule 7.1 | Required | :x: | :white_check_mark: | :x: |
| Rule 7.2 | Required | :x: | :white_check_mark: | :x: |
| Rule 7.3 | Required | :x: | :white_check_mark: | :x: |
| Rule 7.4 | Required | :x: | :white_check_mark: | :x: |
| Rule 8.1 | Required | :x: | :white_check_mark: | :x: |
| Rule 8.2 | Required | :x: | :white_check_mark: | :x: |
| Rule 8.3 | Required | :x: | :white_check_mark: | :x: |
| Rule 8.4 | Required | :x: | :white_check_mark: | :x: |
| Rule 8.5 | Required | :x: | :white_check_mark: | :x: |
| Rule 8.6 | Required | :x: | :white_check_mark: | :x: |
| Rule 8.7 | Advisory | :x: | :white_check_mark: | :x: |
| Rule 8.8 | Required | :x: | :white_check_mark: | :x: |
| Rule 8.9 | Advisory | :x: | :white_check_mark: | :x: |
| Rule 8.10 | Required | :x: | :white_check_mark: | :x: |
| Rule 8.11 | Advisory | :x: | :white_check_mark: | :x: |
| Rule 8.12 | Required | :x: | :white_check_mark: | :x: |
| Rule 8.13 | Advisory | :x: | :white_check_mark: | :x: |
| Rule 8.14 | Required | :x: | :white_check_mark: | :x: |
| Rule 9.1 | Mandatory | :x: | :white_check_mark: | :x: |
| Rule 9.2 | Required | :x: | :white_check_mark: | :x: |
| Rule 9.3 | Required | :x: | :white_check_mark: | :x: |
| Rule 9.4 | Required | :x: | :white_check_mark: | :x: |
| Rule 9.5 | Required | :x: | :white_check_mark: | :x: |
| Rule 10.1 | Required | :x: | :white_check_mark: | :x: |
| Rule 10.2 | Required | :x: | :white_check_mark: | :x: |
| Rule 10.3 | Required | :x: | :white_check_mark: | :x: |
| Rule 10.4 | Required | :x: | :white_check_mark: | :x: |
| Rule 10.5 | Advisory | :x: | :white_check_mark: | :x: |
| Rule 10.6 | Required | :x: | :white_check_mark: | :x: |
| Rule 10.7 | Required | :x: | :white_check_mark: | :x: |
| Rule 10.8 | Required | :x: | :white_check_mark: | :x: |
| Rule 11.1 | Required | :x: | :white_check_mark: | :x: |
| Rule 11.2 | Required | :x: | :white_check_mark: | :x: |
| Rule 11.3 | Required | :x: | :white_check_mark: | :x: |
| Rule 11.4 | Advisory | :x: | :white_check_mark: | :x: |
| Rule 11.5 | Advisory | :x: | :white_check_mark: | :x: |
| Rule 11.6 | Required | :x: | :white_check_mark: | :x: |
| Rule 11.7 | Required | :x: | :white_check_mark: | :x: |
| Rule 11.8 | Required | :x: | :white_check_mark: | :x: |
| Rule 11.9 | Required | :x: | :white_check_mark: | :x: |
| Rule 12.1 | Advisory | :x: | :white_check_mark: | :x: |
| Rule 12.2 | Required |:x: | :white_check_mark: | :x: |
| Rule 12.3 | Advisory | :x: | :white_check_mark: | :x: |
| Rule 12.4 | Advisory | :x: | :white_check_mark: | :x: |
| Rule 13.1 | Required | :x: | :white_check_mark: | :x: |
| Rule 13.2 | Required | :x: | :white_check_mark: | :x: |
| Rule 13.3 | Advisory | :x: | :white_check_mark: | :x: |
| Rule 13.4 | Advisory | :x: | :white_check_mark: | :x: |
| Rule 13.5 | Required | :x: | :white_check_mark: | :x: |
| Rule 13.6 | Mandatory | :x: | :white_check_mark: | :x: |
| Rule 14.1 | Required | :x: | :white_check_mark: | :x: |
| Rule 14.2 | Required | :x: | :white_check_mark: | :x: |
| Rule 14.3 | Required | :x: | :white_check_mark: | :x: |
| Rule 14.4 | Required | :x: | :white_check_mark: | :x: |
| Rule 15.1 | Advisory | :x: | :white_check_mark: | :x: |
| Rule 15.2 | Required | :x: | :white_check_mark: | :x: |
| Rule 15.3 | Required | :x: | :white_check_mark: | :x: |
| Rule 15.4 | Advisory | :x: | :white_check_mark: | :x: |
| Rule 15.5 | Advisory | :x: | :white_check_mark: | :x: |
| Rule 15.6 | Required | :x: | :white_check_mark: | :x: |
| Rule 15.7 | Required | :x: | :white_check_mark: | :x: |
| Rule 16.1 | Required | :x: | :white_check_mark: | :x: |
| Rule 16.2 | Required | :x: | :white_check_mark: | :x: |
| Rule 16.3 | Required | :x: | :white_check_mark: | :x: |
| Rule 16.4 | Required | :x: | :white_check_mark: | :x: |
| Rule 16.5 | Required | :x: | :white_check_mark: | :x: |
| Rule 16.6 | Required | :x: | :white_check_mark: | :x: |
| Rule 16.7 | Required | :x: | :white_check_mark: | :x: |
| Rule 17.1 | Required | :x: | :white_check_mark: | :x: |
| Rule 17.2 | Required | :x: | :white_check_mark: | :x: |
| Rule 17.3 | Mandatory | :x: | :x: | :white_check_mark: |
| Rule 17.4 | Mandatory | :x: | :white_check_mark: | :x: |
| Rule 17.5 | Advisory | :x: | :white_check_mark: | :x: |
| Rule 17.6 | Mandatory | :x: | :white_check_mark: | :x: |
| Rule 17.7 | Required | :x: | :white_check_mark: | :x: |
| Rule 17.8 | Advisory | :x: | :white_check_mark: | :x: |
| Rule 18.1 | Required | :x: | :white_check_mark: | :x: |
| Rule 18.2 | Required | :x: | :white_check_mark: | :x: |
| Rule 18.3 | Required | :x: | :white_check_mark: | :x: |
| Rule 18.4 | Advisory | :x: | :white_check_mark: | :x: |
| Rule 18.5 | Required | :x: | :white_check_mark: | :x: |
| Rule 18.6 | Required | :x: | :white_check_mark: | :x: |
| Rule 18.7 | Required | :x: | :white_check_mark: | :x: |
| Rule 18.8 | Required | :x: | :white_check_mark: | :x: |
| Rule 19.1 | Mandatory | :x: | :white_check_mark: | :x: |
| Rule 19.2 | Advisory | :x: | :white_check_mark: | :x: |
| Rule 20.1 | Advisory | :x: | :white_check_mark: | :x: |
| Rule 20.2 | Required | :x: | :white_check_mark: | :x: |
| Rule 20.3 | Required | :x: | :white_check_mark: | :x: |
| Rule 20.4 | Required | :x: | :white_check_mark: | :x: |
| Rule 20.5 | Advisory | :x: | :white_check_mark: | :x: |
| Rule 20.6 | Required | :x: | :white_check_mark: | :x: |
| Rule 20.7 | Required | :x: | :white_check_mark: | :x: |
| Rule 20.8 | Required | :x: | :white_check_mark: | :x: |
| Rule 20.9 | Required | :x: | :white_check_mark: | :x: |
| Rule 20.10 | Advisory | :x: | :white_check_mark: | :x: |
| Rule 20.11 | Required | :x: | :white_check_mark: | :x: |
| Rule 20.12 | Required | :x: | :white_check_mark: | :x: |
| Rule 20.13 | Required | :x: | :white_check_mark: | :x: |
| Rule 20.14 | Required | :x: | :white_check_mark: | :x: |
| Rule 21.1 | Required | :x: | :white_check_mark: | :x: |
| Rule 21.2 | Required | :x: | :white_check_mark: | :x: |
| Rule 21.3 | Required | :x: | :white_check_mark: | :x: |
| Rule 21.4 | Required | :x: | :white_check_mark: | :x: |
| Rule 21.5 | Required | :x: | :white_check_mark: | :x: |
| Rule 21.6 | Required | :x: | :white_check_mark: | :x: |
| Rule 21.7 | Required | :x: | :white_check_mark: | :x: |
| Rule 21.8 | Required | :x: | :white_check_mark: | :x: |
| Rule 21.9 | Required | :x: | :white_check_mark: | :x: |
| Rule 21.10 | Required | :x: | :white_check_mark: | :x: |
| Rule 21.11 | Required | :x: | :white_check_mark: | :x: |
| Rule 21.12 | Required | :x: | :white_check_mark: | :x: |
| Rule 22.1 | Required | :x: | :white_check_mark: | :x: |
| Rule 22.2 | Mandatory | :x: | :white_check_mark: | :x: |
| Rule 22.3 | Required | :x: | :white_check_mark: | :x: |
| Rule 22.4 | Mandatory | :x: | :white_check_mark: | :x: |
| Rule 22.5 | Mandatory | :x: | :white_check_mark: | :x: |
| Rule 22.6 | Mandatory | :x: | :white_check_mark: | :x: |

## Deviations

No deviations are documented at this time.
