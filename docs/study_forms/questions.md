# Deployment Study Instruments — Question Battery

Five instruments. Likert items are 5-point (1 = Strongly Disagree, 5 =
Strongly Agree) unless marked as intensity scales. Wording of items that also
appear in the paper draft matches `docs/feeding-deployment-docs/src/deployment.tex`;
new items are marked [NEW]. Review all wording with the CR before day 1 (CBPR),
then freeze — daily wording must stay identical for the whole deployment.

---

## Form 1 — Before Study (baseline; administer BEFORE deployment day 1)

**Expectations (prospective TAM)** [NEW]
1. I expect the meal-assistance system to make me more independent in eating.
2. I expect the system to be easy to use.
3. I expect that using the system to improve my independence will be a good idea.
4. If I had access to this system, I expect I would use it in my daily life.
5. I expect using the system to be enjoyable.

**Baseline mealtime (caregiver-assisted; refers to a typical current meal)** [NEW]
6. I feel safe during a typical meal with my caregiver. 
7. I am satisfied with how a typical meal with my caregiver goes.
8. I feel in control of my feeding experience when assisted by my caregiver.
   *(same wording as the end-of-study caregiver-control item → pre/post pair)*
9. I feel a sense of independence when I receive assistance from my caregiver.
   *(same wording as end-of-study caregiver-independence item)*

**Open-ended** [NEW]
10. What do you expect the robot to do well?
11. What do you expect the robot to struggle with?
12. What worries you most about the coming month? What excites you most?
13. Briefly describe a typical mealtime today (who helps, how long it takes,
    how it feels). (paragraph)

---

## Form 2 — Before Meal (daily, ~20 seconds; MUST be answered before "Start Meal")

1. Day number (short answer, filled by researcher/CR)
2. How is your energy right now? (intensity 1 = very low … 5 = very high) [NEW]
3. How hurried do you feel today? (intensity 1 = not at all … 5 = very hurried) [NEW]
4. Anything notable about today? (short answer, optional) [NEW]

*Analysis note: these are a validation probe of the latent affective state and
are WITHHELD from the system (never shown to the preference predictor).*

---

## Form 3 — After Meal (daily)

1. Day number (short answer)
2. I felt safe during today's meal. *(paper item, statement form)*
3. I was satisfied with the robot's performance during today's meal. *(paper item)*
4. I trusted the robot to do the right thing during today's meal. [NEW]
5. I felt in control of how today's meal went. [NEW]
6. What, if anything, did you learn about the robot today, or how did your
   interaction with it change? (paragraph) *(paper item)*
7. Only if something went wrong today (skip otherwise): today's problem changed
   how much I trust the robot. (1 = trust much less … 5 = trust much more)
   [NEW, optional]

---

## Form 4 — After Week (weekly, ×4; capture of the recorded mini-interview)

1. Week number (short answer)
2. What do you do differently with the robot now compared to a week ago?
3. What does the robot do differently for you now?
4. Any tricks or workarounds you've invented this week?
5. What have you stopped checking or worrying about?
6. Is there anything you've given up on, or stopped asking the robot to do?
7. If the robot broke tomorrow, what would you miss most from this week?

(All paragraph fields. The conversation is audio-recorded; this form is the
structured capture. FSS and OT instruments are administered separately by the
OT — not in this form.)

---

## Form 5 — After Study (end of deployment)

**Technology Acceptance (paper items)**
1. Using this meal-assistance system will make me more independent in eating.
2. This meal-assistance system is easy to use.
3. Using the meal-assistance system for improving my independence is a good idea.
4. Assuming I have access to this meal-assistance system, I predict that I
   would use it in my daily life.
5. I find using this meal-assistance system to be enjoyable.

**Control and Independence (paper items; caregiver + robot versions)**
6. I feel in control of my feeding experience when assisted by my caregiver.
7. I feel in control of my feeding experience when assisted by the robot.
8. I feel a sense of independence when I receive assistance from my caregiver.
9. I feel a sense of independence when I receive assistance from the robot.

**Co-Adaptation (paper items)**
10. The robot got better at understanding my preferences over the course of the study.
11. I got better at working with the robot over the course of the study.

**Predictability / mental model** [NEW]
12. By the end of the study, I could predict what the robot would do next.
13. I knew what the robot could and could not do.

**Agency calibration** [NEW]
14. There were times I wanted more control than the robot gave me.
15. There were times I wished the robot would act without asking me.

**Comparative burden** [NEW]
16. Compared to the first week, mealtimes with the robot required less of my effort.

**Open-ended (backup capture; full exit interview is verbal + recorded)** [NEW]
17. What changed most between your first week and your last week with the robot?
18. What did you teach the robot — and what did it teach you?
19. What would you tell the next person who gets this robot?

---

## Administration notes

- Daily forms: identical wording every day, never edited mid-deployment.
- Pre-meal form answered BEFORE pressing Start Meal (otherwise the state probe
  is contaminated by the meal).
- Responses export from the linked Google Sheet at teardown into
  `daily_survey.jsonl` alongside the day bundle (see deployment_logging_plan.md).
- Day-number field is the join key to the robot logs; the Forms timestamp is a
  secondary check.
- Exit interview protocol (verbal, recorded) is separate from Form 5:
  week-1-vs-week-4 walkthrough; a moment trust dropped and what rebuilt it;
  what you taught it / it taught you; advice for the next user; would you keep
  it, and under what conditions.
