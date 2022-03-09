FROM erdosproject/pylot:v0.3.3

WORKDIR /home/erdos/workspace/pylot

RUN git clone https://github.com/eclipse-zenoh/zenoh-python.git 

WORKDIR zenoh-python 

RUN git checkout 7b566a07901468d8572426c8c5241b3e551792bf

RUN pip install .

WORKDIR /home/erdos/workspace/pylot

COPY carla_source_operator.py .

CMD python carla_source_operator.py 

