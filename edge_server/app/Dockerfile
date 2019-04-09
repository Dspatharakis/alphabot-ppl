FROM python:2.7.13
ADD ./requirements.txt ./

RUN echo "deb [check-valid-until=no] http://archive.debian.org/debian jessie-backports main" > /etc/apt/sources.list.d/jessie-backports.list

RUN sed -i '/deb http:\/\/deb.debian.org\/debian jessie-updates main/d' /etc/apt/sources.list

RUN apt-get -o Acquire::Check-Valid-Until=false update

RUN pip install --upgrade pip \
 && pip install -r ./requirements.txt \
&& rm -rf /var/lib/apt/lists/*

ADD . ./

COPY entrypoint.sh /entrypoint.sh 
RUN chmod +x /entrypoint.sh


ENTRYPOINT /entrypoint.sh



