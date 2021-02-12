#include <ESPFileUtils.h>

boolean mountFileSystem()
{
  if (!LittleFS.begin())
  {
    Serial.println("LittleFS mount failed");
    return false;
  }
  return true;
}

boolean fileExists(String path)
{
  Serial.println("Checking if file exists: " + path);
  File file = LittleFS.open(path, "r");
  if (file)
  {
    Serial.println("file exists: " + path);
    file.close();
    return true;
  }
  Serial.println("file not exists: " + path);
  return false;
}

String readFile(const String path)
{
  Serial.println("Reading file: " + path);

  String fileContent = "";
  File file = LittleFS.open(path, "r");
  if (!file)
  {
    Serial.println("Failed to open file for reading");
    return fileContent;
  }

  Serial.print("Read from file: ");
  fileContent = file.readString();
  Serial.println(fileContent);

  file.close();
  return fileContent;
}

void writeFile(String path, String message)
{
  Serial.println("Writing file: " + path);

  File file = LittleFS.open(path, "w");
  if (!file)
  {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message))
  {
    Serial.println("File written");
  }
  else
  {
    Serial.println("Write failed");
  }
  file.close();
}

boolean deleteFile(const String path)
{
  Serial.println("Deleting file: " + path);

  if (LittleFS.remove(path))
  {
    Serial.println("File deleted");
    return true;
  }
  Serial.println("Delete failed");
  return false;
}
