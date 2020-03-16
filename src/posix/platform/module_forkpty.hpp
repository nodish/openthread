class Forkpty : public FileDescriptor
{
private:
    int OpenFile(const char *aDevice, Arguments &aArguments);
};

