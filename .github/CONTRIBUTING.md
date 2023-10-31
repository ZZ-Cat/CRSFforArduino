# Contributing to CRSF for Arduino

Thank you for helping to make CRSF for Arduino better.  
Before you can go Gung Ho on coding, there are a few house rules I need you to familiarise yourself with.

## Do's and don'ts of CRSF for Arduino

### **Do...**

1. Read the README in its entirety _before_ submitting an issue, as your "bug" may very well be a case of PICNIC.
2. Respond in a timely and respectful manner.
3. If something isn't working, please provide evidence - EG Screenshots of compiler errors.
4. Provide concise details.
5. For general feedback, comments and questions, use the [Discussions Tab](https://github.com/ZZ-Cat/CRSFforArduino/discussions).
6. Format your code. CRSF for Arduino uses clang-format as a formatter. So, use that if you actually want to contribute to CRSF for Arduino's development.

### **Don't...**

1. Use CRSF for Arduino on incompatible hardware and write me an issue about "strange behaviour".
2. Ghost me for more than 14 consecutive days.
3. Write things like "This isn't working" and refuse to provide any information &/or context beyond that.
4. Write walls of text. Do this and watch how quickly I mark your issue as "TL;DR" and close it without any response from me.
5. Beg for updates. Please don't do this. If you do, I will intentionally take longer to release said update to spite you.
6. Assume that the project is "dead" after any significant amount of time has passed.

## A few notable points

- CRSF for Arduino requires _all_ commits to be properly signed and verfied.  
  This means that you _must_ have commit signing with either an SSH or GPG key set-up.
  Pull Requests with unverified commits _will not_ be accepted.
- Not every Pull Request can or will be accepted.  
  Simply because you submitted it, that doesn't automatically guarantee that your Pull Request will be merged.  
  Generally speaking, I will let you know as to why your Pull Request will not be merged. Nine-times-out-of-ten, it's because of something that's easily fixable within reason.
- I reserve the right to decline any Pull Request(s) if I deem it necessary to do so.
- All Pull Requests to the Main-Trunk _must_ be reviewed before they can be merged.
  If I suggest to you to make changes to your Pull Request, don't fight me on it. I already have a reason why those changes need to be made and I would have told you what that reason is. If you are having difficulty with something, let me know.
- All checks on your Pull Request _must_ pass before it can be merged.
  These checks ensure correct code formatting, and the codebase compiles successfully on all supported development boards.

## Ghosting

### What is it?

This describes a situation where I have not received any interaction from you over a signifcant amount of time.  
I define "a significant amount of time" to be three weeks or more.  
For example, if you open an issue, and I ask for clarification on said issue and you have not replied to that message over a significant amount of time. This is ghosting.  
Another example is I tag you in a Pull Request because I require your assistance, yet there has not been any dialog from you over a significant amount of time. That is also ghosting.

### Why is ghosting bad?

Short answer: It's unprofessional.  

Consider this scenario:  
You submit an Issue, requesting compatibility for your development board.  
I don't have the board-in-question available to me on-hand to test on my own end, so I elect to enlist your assistance.  
I go ahead and open a Pull Request, tagging you in it (and asking you to test the Pull Request).  
Only...  
One week passes. Okay, they're probably busy with life stuff and maybe they will respond over the weekend.  
Weekend comes... still no response. Okay, I will give them another week.  
Second week comes... still no response. Maybe if I give them another week? Hopefully they might respond?.
Third week goes by... still no response, and CRSF for Arduino's development has stalled.

### How I combat it

For a start, I have a grace period of up to 14 days past the last interaction you have had with me.  
After the 14 days is up, I decide (at my discretion) to close your Issue or Pull Request. In the context of a Pull Request, depending on the state of the code (EG Accuracy, completeness and functionality), at my discretion, I may merge the Pull Request "as-is", and any potential bugs that it could introduce (that I may have missed), you are welcome to open an Issue about it later on.

This grace period exists to mitigate stalled development, and keep the development of CRSF for Arduino moving along.  
People that genuinely want to help out with the project, they typically respond within 24~72 hours, with some outlying exceptions responding up to 7~12 days apart.  
This grace period strikes a balance between my need to keep the project moving, and giving someone time to do what they need to do in their own private life as well as contributing to CRSF for Arduino.

## For more information

- [About forks](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/working-with-forks/about-forks)
- [Creating a Pull Request from your fork](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request-from-a-fork)
- [Code of Conduct](/CODE_OF_CONDUCT.md)
- [Submit an Issue](https://github.com/ZZ-Cat/CRSFforArduino/issues/new/choose)
- [License](/LICENSE.md)
- [README](/README.md)
