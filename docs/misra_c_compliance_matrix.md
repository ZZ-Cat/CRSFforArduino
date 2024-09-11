# MISRA C Compliance and Deviations

This document outlines the MISRA C compliance of CRSF for Arduino and documents any deviations from the MISRA C guidelines.  
The [compliance](#compliance) section lists all of the MISRA C guidelines that CRSF for Arduino is compliant with.
The [deviations](#deviations) section lists any deviations from the MISRA C guidelines.

## Compliance Matrix

The following tables represent the compliance level of CRSF for Arduino with the MISRA C guidelines, and the tools used to verify compliance.  
The compliance level is determined by the following flags:

- **Advisory**: CRSF for Arduino should follow this guideline wherever reasonable and practical, unless the relevant deviation is documented.
- **Required**: CRSF for Arduino should follow this guideline without any deviations unless that deviation is already documented.
- **Mandatory**: CRSF for Arduino must follow this guideline. Deviations are not permitted.

The icons listed here are used in the [directives](#directives) and [rules](#rules) tables to represent the compliance level.
An icon is placed in a cell that corresponds to the guideline and the method used to verify compliance. Each icon represents a different compliance level.
The icons are as follows:

- :white_check_mark: - CRSF for Arduino is compliant with the guideline.
- :negative_squared_cross_mark: - CRSF for Arduino violates the guideline. A deviation from the guideline is being reviewed and may be documented.
- :o: - A deviation from the guideline is documented. See [deviations](#deviations) for more information.
- :no_entry_sign: - CRSF for Arduino violates the guideline and is yet to be fixed.
- :x: - This method is not used.
- :warning: - Rule status is unknown and requires manual review.

### Directives

| MISRA Directive | Rule flag | Manual review |
| --- | --- | --- |
| Directive 1.1 | Required | :warning: |
| Directive 2.1 | Required | :no_entry_sign: |
| Directive 3.1 | Required | :warning: |
| Directive 4.1 | Required | :white_check_mark: |
| Directive 4.2 | Advisory | :white_check_mark: |
| Directive 4.3 | Required | :white_check_mark: |
| Directive 4.4 | Advisory | :warning: |
| Directive 4.5 | Advisory | :white_check_mark: |
| Directive 4.6 | Advisory | :white_check_mark: |
| Directive 4.7 | Required | :white_check_mark: |
| Directive 4.8 | Advisory | :warning: |
| Directive 4.9 | Advisory | :white_check_mark: |
| Directive 4.10 | Required | :white_check_mark: |
| Directive 4.11 | Required | :no_entry_sign: |
| Directive 4.12 | Required | :no_entry_sign: |
| Directive 4.13 | Advisory | :warning: |

### Rules

| MISRA Rule | Rule flag | GCC Arm None EABI | Cppcheck | Manual review |
| --- | --- | --- | --- | --- |
| Rule 1.1 | Required | :x: | :x: | :warning: |
| Rule 1.2 | Required | :x: | :x: | :warning: |
| Rule 1.3 | Required | :x: | :white_check_mark: | :x: |
| Rule 2.1 | Required | :x: | :white_check_mark: | :x: |
| Rule 2.2 | Required | :x: | :white_check_mark: | :x: |
| Rule 2.3 | Advisory | :x: | :no_entry_sign: | :x: |
| Rule 2.4 | Advisory | :x: | :no_entry_sign: | :x: |
| Rule 2.5 | Advisory | :x: | :no_entry_sign: | :x: |
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
| Rule 5.8 | Required | :x: | :no_entry_sign: | :x: |
| Rule 5.9 | Required | :x: | :white_check_mark: | :x: |
| Rule 6.1 | Required | :x: | :white_check_mark: | :x: |
| Rule 6.2 | Required | :x: | :white_check_mark: | :x: |
| Rule 7.1 | Required | :x: | :white_check_mark: | :x: |
| Rule 7.2 | Required | :x: | :white_check_mark: | :x: |
| Rule 7.3 | Required | :x: | :white_check_mark: | :x: |
| Rule 7.4 | Required | :x: | :white_check_mark: | :x: |
| Rule 8.1 | Required | :x: | :white_check_mark: | :x: |
| Rule 8.2 | Required | :x: | :no_entry_sign: | :x: |
| Rule 8.3 | Required | :x: | :white_check_mark: | :x: |
| Rule 8.4 | Required | :x: | :no_entry_sign: | :x: |
| Rule 8.5 | Required | :x: | :white_check_mark: | :x: |
| Rule 8.6 | Required | :x: | :no_entry_sign: | :x: |
| Rule 8.7 | Advisory | :x: | :no_entry_sign: | :x: |
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
| Rule 10.4 | Required | :x: | :no_entry_sign: | :x: |
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
| Rule 12.1 | Advisory | :x: | :no_entry_sign: | :x: |
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
| Rule 17.3 | Mandatory | :x: | :x: | :warning: |
| Rule 17.4 | Mandatory | :x: | :white_check_mark: | :x: |
| Rule 17.5 | Advisory | :x: | :white_check_mark: | :x: |
| Rule 17.6 | Mandatory | :x: | :white_check_mark: | :x: |
| Rule 17.7 | Required | :x: | :white_check_mark: | :x: |
| Rule 17.8 | Advisory | :x: | :no_entry_sign: | :x: |
| Rule 18.1 | Required | :x: | :white_check_mark: | :x: |
| Rule 18.2 | Required | :x: | :white_check_mark: | :x: |
| Rule 18.3 | Required | :x: | :white_check_mark: | :x: |
| Rule 18.4 | Advisory | :x: | :white_check_mark: | :x: |
| Rule 18.5 | Required | :x: | :white_check_mark: | :x: |
| Rule 18.6 | Required | :x: | :white_check_mark: | :x: |
| Rule 18.7 | Required | :x: | :white_check_mark: | :x: |
| Rule 18.8 | Required | :x: | :white_check_mark: | :x: |
| Rule 19.1 | Mandatory | :x: | :white_check_mark: | :x: |
| Rule 19.2 | Advisory | :x: | :no_entry_sign: | :x: |
| Rule 20.1 | Advisory | :x: | :white_check_mark: | :x: |
| Rule 20.2 | Required | :x: | :white_check_mark: | :x: |
| Rule 20.3 | Required | :x: | :white_check_mark: | :x: |
| Rule 20.4 | Required | :x: | :white_check_mark: | :x: |
| Rule 20.5 | Advisory | :x: | :white_check_mark: | :x: |
| Rule 20.6 | Required | :x: | :white_check_mark: | :x: |
| Rule 20.7 | Required | :x: | :white_check_mark: | :x: |
| Rule 20.8 | Required | :x: | :white_check_mark: | :x: |
| Rule 20.9 | Required | :x: | :no_entry_sign: | :x: |
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
