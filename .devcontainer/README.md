# Windows Setup

https://www.jetbrains.com/help/idea/using-ssh-keys.html#Troubleshooting

In a root powershell

```commandline
Set-Service ssh-agent -StartupType Automatic
Start-Service ssh-agent
Get-Service ssh-agent
```

Then add the ssh key

```commandline
ssh-add $HOME/.ssh/<your ssh key>
```