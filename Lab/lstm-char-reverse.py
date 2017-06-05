#!/home/william/anaconda3/envs/tensorflow/bin/python
import random
import string
import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt

def input_target_generator(min_duration=5, max_duration=10, length=3, batch_size=1):
    while True:
        x = []
        y = []
        for i in range(batch_size):
            # duration = np.random.randint(min_duration, max_duration)
            # inputs = np.random.randn(duration).astype(np.float32)
            inputs = np.random.randint(10, size=length)
            targets = np.cumsum(inputs).astype(np.float32)
            x.append(inputs)
            y.append(targets)
        yield np.stack(x), np.stack(y)

char_id_dict = {char: idx for idx, char in enumerate(string.ascii_lowercase)}
id_char_dict = {idx: char for idx, char in enumerate(string.ascii_lowercase)}

def char2id(strArray):
    return map(lambda c:char_id_dict[c], strArray)

def id2char(intArray):
    return map(lambda c:id_char_dict[c], intArray)

def random_chars(length):
    chars = ''.join(random.choice(string.ascii_lowercase) for _ in range(length))
    chars = list(chars)
    chars = list(char2id(chars))
    return chars, chars[::-1]

def random_string_generator(batch_size=128, chars=5):
    while True:
        xy = [random_chars(chars) for _ in range(batch_size)]
        x, y = zip(*((pair[0], pair[1]) for pair in xy))
        yield np.array(x).reshape(batch_size, -1), np.array(y).reshape(batch_size, -1)


def run_test():
    # http://blog.gaurav.im/2017/01/11/a-gentle-intro-to-recurrent-nns-in-tensorflow/
    tf.reset_default_graph()
    batch_size = 2
    sequence_length = 4
    input_size = 26  # no. of chars in onehot
    hidden_size = 256  # no. of hidden units

    tf_x = tf.placeholder(tf.int32, shape=[batch_size, sequence_length], name="x")
    tf_y = tf.placeholder(tf.int32, shape=[batch_size, sequence_length], name="y")
    tf_x_oh = tf.cast(tf.one_hot(tf_x, depth=input_size), tf.float32)  # [batch, sequence_length, 26]
    tf_y_oh = tf.cast(tf.one_hot(tf_y, depth=input_size), tf.float32)  # [batch, sequence_length, 26]

    cell = tf.contrib.rnn.BasicLSTMCell(num_units=hidden_size)
    rnn_outputs, state = tf.nn.dynamic_rnn(cell, inputs=tf_x_oh, dtype=tf.float32)

    class MyHelper(tf.contrib.seq2seq.Helper):
        def __init__(self):
            self._batch_size = batch_size

        @property
        def batch_size(self):
            return self._batch_size

        # returns (false, first_input)
        def initialize(self):
            finished = tf.equal(0, sequence_length)
            all_finished = tf.reduce_all(finished)
            all_zeros = tf.zeros([batch_size, input_size])
            return (all_finished, all_zeros) # () -> (finished, next_inputs)

        # returns (batch_size, input)
        def sample(self, time, outputs, state, name=None):
            sample_ids = tf.cast(tf.argmax(outputs, axis=-1), tf.int32)
            return sample_ids  # (time, outputs, state) -> sample_ids

        # returns (finished, (batch_size, input_size), (1, hidden_size))
        def next_inputs(self, time, outputs, state, sample_ids, name=None):
            next_time = time+1
            finished = tf.reduce_all(tf.equal(next_time, sequence_length))
            out = get_logits(outputs)
            out = tf.reshape(out, [batch_size, input_size])
            return finished, out, state # (time, outputs, state, sample_ids) -> (finished, next_inputs, next_state)

    decode_cell = tf.contrib.rnn.BasicLSTMCell(num_units=hidden_size)
    helper = tf.contrib.seq2seq.TrainingHelper(inputs=tf_y_oh, sequence_length=[sequence_length])
    my_helper = MyHelper()

    def decode(helper):
        with tf.variable_scope('decoder') as scope:
            my_decoder = tf.contrib.seq2seq.BasicDecoder(cell=decode_cell, helper=helper, initial_state=state)
            decoder_ouputs, final_state = tf.contrib.seq2seq.dynamic_decode(my_decoder)
            decoder_logits = decoder_ouputs.rnn_output  # (batch, seq_length, hidden)
            decoder_sample_ids = decoder_ouputs.sample_id
            logits = get_logits(decoder_logits)
            predictions = tf.argmax(logits, axis=2)
        return logits, predictions

    # Why = tf.Variable(tf.random_normal(shape=[batch_size, hidden_size, input_size], dtype=tf.float32))
    # by = tf.Variable(tf.random_normal(shape=[batch_size, 1, input_size], dtype=tf.float32))

    def get_logits(output):
        with tf.variable_scope('logits') as scope:
            logits = tf.layers.dense(output, input_size) #output 26 one_hot
            # logits = tf.matmul(output, Why) + by
        return logits

    logits, predictions = decode(my_helper)

    # inference_logits, inference_predictions = decode(my_helper)
    loss = tf.nn.softmax_cross_entropy_with_logits(logits=logits, labels=tf_y_oh)
    learning_rate = 1e-4
    optimizer = tf.train.AdamOptimizer(learning_rate).minimize(loss)

    with tf.Session() as sess:
        sess.run(tf.global_variables_initializer())

        for epoch in range(50001):
            randomStringGenerator = random_string_generator(batch_size=batch_size, chars=sequence_length)
            x, y = next(randomStringGenerator)
            feed_dict = {tf_x: x, tf_y: y}
            o, l, _ = sess.run([predictions, loss, optimizer], feed_dict=feed_dict)
            # print(o)

            if (epoch % 1000 == 0):
                print('run {0} --------'.format(epoch))
                print('target:', y)
                print('output:', o)
                print('loss:', l)
                # o, l, _ = sess.run([inference_predictions, loss, optimizer], feed_dict=feed_dict)
                # print('target:', y)
                # print('inference output:', o)
                # print('inference loss:', l)


if __name__ == '__main__':
#Testing generator
    # gen = input_target_generator(batch_size=2)
    # x, y = next(gen)
    # print(x)
    # print(y)

#Test the RNN
    run_test()
    # print(tf.__version__)
    # a = np.arange(8).reshape(2, 2, 2)
    # print(a)
    # print('')
    # print(np.argmax(a, axis=-1))