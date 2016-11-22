#include <iostream>

void sendCommand(unsigned char *cmd, int cnt)
{
	char tmp[200];
	int tmp_cnt = 0;
	tmp[tmp_cnt++] = ':';

	int sum = 0;
	for(int i = 0; i < cnt; i++)
	{
		sum += cmd[i];
		tmp_cnt+= sprintf(tmp+tmp_cnt, "%02X", cmd[i]);
	}
	tmp_cnt+= sprintf(tmp+tmp_cnt, "%02X", (0x100 - sum) & 0xff);
	tmp[tmp_cnt++] = 0x0D;
	tmp[tmp_cnt++] = 0x0A;

	std::cout << "setVelCmd (" << tmp_cnt << ") : " << tmp << std::endl;
}

int setVelCmd(int vel, unsigned char *cmd)
{
	int cnt = 0;
	cmd[cnt++] = 0x11;
	cmd[cnt++] = 0x10;
	cmd[cnt++] = 0x00;
	cmd[cnt++] = 0x18;
	cmd[cnt++] = 0x00;
	cmd[cnt++] = 0x02;
	cmd[cnt++] = 0x04;

	cmd[cnt++] = (vel & 0xffff) >> 8;
	cmd[cnt++] = (vel & 0xffff) & 0xff;
	cmd[cnt++] = (vel >> 16) >> 8;
	cmd[cnt++] = (vel >> 16) & 0xff;

	return cnt;
}

int main()
{
	unsigned char cmd[200];
	int n = 0;

	n = setVelCmd(200, cmd);
	sendCommand(cmd, n);

	return 0;
}
