# Test documentation and results
Find here the test procedures used during the developpement of SecurBot.
Each test is documented in a file like `template_test_SEC-XXX.rtf`

The rtf format is used for interoperability and minimal git diffs.
See [editing RTFs](#Editing-RTF-files) for more information.

### Create a new test
* Copy the `template_test_SEC-XXX.rtf` and rename it for the purposes to the test title.
* Place file under this directory, create subdirectories for components being test.
* Fill in form with necessary information.
* Commit test description and procedure along with necessary annexed files for documentation. (ie. illustrations, schmatics, settings)

### Approving tests
* Assign someone to review created tests.
* Reviewer commits to confirm approval, indicating his name and date of the approval.

### Executing tests
* Make sure the test was reviewed and approved.
* Execute test according to instructed procedure.
* Fill in the *Test Results* section with a new entry for every test run.
* Include output files and or other test-relevant files if necessary.
* Commit updated test `<test_title>_SEC-XXX.rtf` along with relevant extra files.

### Editing RTF files
Avoid using text editors such as Microsoft Word or LibreOffice writer to edit test files.
They tend to include additionnal tags, considerably increasing diff size and crippling raw file readability.
On Windows, consider WordPad, also available through WINE for Linux users.
Alternatively please use a text editor, or simple word processor.
