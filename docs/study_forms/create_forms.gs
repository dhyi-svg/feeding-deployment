/**
 * create_forms.gs -- creates the five deployment study forms in Google Drive
 * and links them all to one response spreadsheet.
 *
 * Usage: script.google.com -> New project -> paste this file -> Run
 * `createDeploymentForms` -> authorize -> open the execution log for the URLs.
 *
 * Wording source of truth: docs/study_forms/questions.md (keep in sync; the
 * daily forms' wording must stay frozen for the whole deployment).
 */

const AGREE = ['Strongly disagree', 'Strongly agree'];

function likert_(form, title, required = true, labels = AGREE) {
  const it = form.addScaleItem().setTitle(title).setBounds(1, 5);
  it.setLabels(labels[0], labels[1]);
  it.setRequired(required);
  return it;
}

function para_(form, title, required = true) {
  return form.addParagraphTextItem().setTitle(title).setRequired(required);
}

function short_(form, title, required = true) {
  return form.addTextItem().setTitle(title).setRequired(required);
}

function header_(form, title) {
  return form.addSectionHeaderItem().setTitle(title);
}

function createDeploymentForms() {
  const ss = SpreadsheetApp.create('[Table for Two] Survey Responses');
  const urls = [];

  // ---------------------------------------------------------- 1. Before Study
  let f = FormApp.create('[Table for Two] 1. Before Study (baseline)');
  f.setDescription(
    'Administer ONCE, before deployment day 1. Expectations + baseline ' +
    '(caregiver-assisted) mealtime experience.');
  header_(f, 'Expectations');
  likert_(f, 'I expect the meal-assistance system to make me more independent in eating.');
  likert_(f, 'I expect the system to be easy to use.');
  likert_(f, 'I expect that using the system to improve my independence will be a good idea.');
  likert_(f, 'If I had access to this system, I expect I would use it in my daily life.');
  likert_(f, 'I expect using the system to be enjoyable.');
  header_(f, 'A typical mealtime today (with your caregiver)');
  likert_(f, 'I feel safe during a typical meal with my caregiver.');
  likert_(f, 'I am satisfied with how a typical meal with my caregiver goes.');
  likert_(f, 'I feel in control of my feeding experience when assisted by my caregiver.');
  likert_(f, 'I feel a sense of independence when I receive assistance from my caregiver.');
  header_(f, 'In your words');
  para_(f, 'What do you expect the robot to do well?');
  para_(f, 'What do you expect the robot to struggle with?');
  para_(f, 'What worries you most about the coming month? What excites you most?');
  para_(f, 'Briefly describe a typical mealtime today (who helps, how long it takes, how it feels).');
  f.setDestination(FormApp.DestinationType.SPREADSHEET, ss.getId());
  urls.push(['Before Study', f.getPublishedUrl(), f.getEditUrl()]);

  // ----------------------------------------------------------- 2. Before Meal
  f = FormApp.create('[Table for Two] 2. Before Meal (daily)');
  f.setDescription(
    'Every meal, BEFORE pressing Start Meal (~20 seconds). ' +
    'These answers are for the study only and are never shown to the robot.');
  short_(f, 'Day number');
  likert_(f, 'How is your energy right now?', true, ['Very low', 'Very high']);
  likert_(f, 'How hurried do you feel today?', true, ['Not at all', 'Very hurried']);
  short_(f, 'Anything notable about today? (optional)', false);
  f.setDestination(FormApp.DestinationType.SPREADSHEET, ss.getId());
  urls.push(['Before Meal', f.getPublishedUrl(), f.getEditUrl()]);

  // ------------------------------------------------------------ 3. After Meal
  f = FormApp.create('[Table for Two] 3. After Meal (daily)');
  f.setDescription('Every meal, after finishing (~90 seconds).');
  short_(f, 'Day number');
  likert_(f, 'I felt safe during today\'s meal.');
  likert_(f, 'I was satisfied with the robot\'s performance during today\'s meal.');
  likert_(f, 'I trusted the robot to do the right thing during today\'s meal.');
  likert_(f, 'I felt in control of how today\'s meal went.');
  para_(f, 'What, if anything, did you learn about the robot today, or how did your interaction with it change?');
  likert_(f,
    'ONLY if something went wrong today (skip otherwise): today\'s problem changed how much I trust the robot.',
    false, ['Trust much less', 'Trust much more']);
  f.setDestination(FormApp.DestinationType.SPREADSHEET, ss.getId());
  urls.push(['After Meal', f.getPublishedUrl(), f.getEditUrl()]);

  // ------------------------------------------------------------ 4. After Week
  f = FormApp.create('[Table for Two] 4. After Week (weekly)');
  f.setDescription(
    'Weekly (x4). Structured capture of the recorded mini-interview. ' +
    'OT-administered instruments (e.g. FSS, COPM) are separate.');
  short_(f, 'Week number');
  para_(f, 'What do you do differently with the robot now compared to a week ago?');
  para_(f, 'What does the robot do differently for you now?');
  para_(f, 'Any tricks or workarounds you\'ve invented this week?');
  para_(f, 'What have you stopped checking or worrying about?');
  para_(f, 'Is there anything you\'ve given up on, or stopped asking the robot to do?');
  para_(f, 'If the robot broke tomorrow, what would you miss most from this week?');
  f.setDestination(FormApp.DestinationType.SPREADSHEET, ss.getId());
  urls.push(['After Week', f.getPublishedUrl(), f.getEditUrl()]);

  // ------------------------------------------------------------ 5. After Study
  f = FormApp.create('[Table for Two] 5. After Study (end of deployment)');
  f.setDescription('Administer once, at the end of the deployment. The verbal exit interview is separate.');
  header_(f, 'Technology acceptance');
  likert_(f, 'Using this meal-assistance system will make me more independent in eating.');
  likert_(f, 'This meal-assistance system is easy to use.');
  likert_(f, 'Using the meal-assistance system for improving my independence is a good idea.');
  likert_(f, 'Assuming I have access to this meal-assistance system, I predict that I would use it in my daily life.');
  likert_(f, 'I find using this meal-assistance system to be enjoyable.');
  header_(f, 'Control and independence');
  likert_(f, 'I feel in control of my feeding experience when assisted by my caregiver.');
  likert_(f, 'I feel in control of my feeding experience when assisted by the robot.');
  likert_(f, 'I feel a sense of independence when I receive assistance from my caregiver.');
  likert_(f, 'I feel a sense of independence when I receive assistance from the robot.');
  header_(f, 'Working together over the month');
  likert_(f, 'The robot got better at understanding my preferences over the course of the study.');
  likert_(f, 'I got better at working with the robot over the course of the study.');
  likert_(f, 'By the end of the study, I could predict what the robot would do next.');
  likert_(f, 'I knew what the robot could and could not do.');
  likert_(f, 'There were times I wanted more control than the robot gave me.');
  likert_(f, 'There were times I wished the robot would act without asking me.');
  likert_(f, 'Compared to the first week, mealtimes with the robot required less of my effort.');
  header_(f, 'In your words');
  para_(f, 'What changed most between your first week and your last week with the robot?');
  para_(f, 'What did you teach the robot -- and what did it teach you?');
  para_(f, 'What would you tell the next person who gets this robot?');
  f.setDestination(FormApp.DestinationType.SPREADSHEET, ss.getId());
  urls.push(['After Study', f.getPublishedUrl(), f.getEditUrl()]);

  // ------------------------------------------------------------------- report
  Logger.log('Response spreadsheet: %s', ss.getUrl());
  urls.forEach(function (u) {
    Logger.log('%s\n  fill:  %s\n  edit:  %s', u[0], u[1], u[2]);
  });
}
