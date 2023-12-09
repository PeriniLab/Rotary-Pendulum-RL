import tensorflow as tf
from tensorflow.python.keras import Sequential
from tensorflow.python.keras.layers import Dense

class DeepQNetwork:
    """
    Deep Q Network to approximate the Q function
    """
    def __init__(self, lr, num_actions, input_dims, fc_dims = [32, 32], opt='adam', loss='mse'):

        self.model = Sequential()
        for i in range(len(fc_dims)):
            if i == 0:
                self.model.add(Dense(fc_dims[i], input_shape=(input_dims,), activation='relu'))
            else:
                self.model.add(Dense(fc_dims[i], activation='relu'))

        self.model.add(Dense(num_actions, activation='linear'))
        self.model.compile(optimizer=opt, loss=loss)
        self.model.optimizer.learning_rate = lr
    
    def predict(self, state):
        """
        Predict the Q values for a given state
        """
        return self.model(state).numpy()

    def train_on_batch(self, states, q_targets):
        """
        Train the network on a batch of states and q_targets
        """
        return self.model.train_on_batch(states, q_targets)
    
    def train_batch_gradientTape(self, states, q_targets):
        """
        Train the network on a batch of states and q_targets using GradientTape
        """
        with tf.GradientTape() as tape:
            predictions = self.model(states)
            loss = tf.keras.losses.MSE(q_targets, predictions)
        gradients = tape.gradient(loss, self.model.trainable_variables)
        self.model.optimizer.apply_gradients(zip(gradients, self.model.trainable_variables))
        # take the mean of the loss
        loss = tf.reduce_mean(loss).numpy()
        return loss

    def evaluate(self, states, q_targets, verbose=0):
        """
        Evaluate the network on a batch of states and q_targets
        """
        return self.model.evaluate(states, q_targets, verbose=verbose)
